from __future__ import annotations

import logging
import math
from pathlib import Path
from typing import Dict, Tuple

from .config import WorkcellConfig
from .assets import AssetCatalog
from .prim_utils import Pose, find_anchorpoints, get_world_pose, set_world_pose
from .controllers.spawner import Spawner
from .controllers.conveyor import ConveyorController, ConveyorPath
from .controllers.safety import SafetyController
from .controllers.state_machine import WorkcellPoints, WorkcellStateMachine
from .analytics.throughput import ThroughputStats

log = logging.getLogger(__name__)


class WorkcellWorld:
    """
    Orchestrates:
    - stage loading
    - discovery of conveyor anchorpoints
    - simulation loop
    """

    def __init__(self, cfg: WorkcellConfig):
        self.cfg = cfg
        self.cfg.validate()

        self.stats = ThroughputStats()
        self._world = None

        self._spawner = None
        self._sm = None
        self._safety = None

        self._pose_cache: Dict[str, Tuple[float, float, float, float]] = {}

    def load(self) -> None:
        self._open_stage(self.cfg.env_usd_path)

        # Create a World after opening the stage
        from omni.isaac.core import World  # type: ignore
        self._world = World(physics_dt=self.cfg.physics_dt, rendering_dt=self.cfg.render_dt, stage_units_in_meters=1.0)
        self._world.reset()

        # Discover conveyor paths using Anchorpoint prims
        a08_path = self._discover_conveyor_path(self.cfg.prims.conveyor_a08)
        a23_path = self._discover_conveyor_path(self.cfg.prims.conveyor_a23)
        a05_path = self._discover_conveyor_path(self.cfg.prims.conveyor_a05)

        conv_a08 = ConveyorController("A08", a08_path, speed_mps=self.cfg.conveyor_speed_mps)
        conv_a23 = ConveyorController("A23", a23_path, speed_mps=self.cfg.conveyor_speed_mps)
        conv_a05 = ConveyorController("A05", a05_path, speed_mps=self.cfg.conveyor_speed_mps)

        # Compute important points
        spawn_a08 = self._offset_z(a08_path.points[0], 0.05)
        start_a23 = self._offset_z(a23_path.points[0], 0.05)
        end_a23 = self._offset_z(a23_path.points[-1], 0.05)
        start_a05 = self._offset_z(a05_path.points[0], 0.05)
        end_a05 = self._offset_z(a05_path.points[-1], 0.05)

        trash_pose = get_world_pose(self.cfg.prims.trash_bin).position
        trash_bin = self._offset_z(trash_pose, 0.25)

        # spawn assembled-step assets near the assembly human
        human_pose = get_world_pose(self.cfg.prims.human_assemble).position
        human_zone = (human_pose[0] + 0.4, human_pose[1], human_pose[2] + 0.1)

        # AS3/AS4 spawn in front of UR5 (use end of A23)
        ur5_zone = (end_a23[0] - 0.2, end_a23[1], end_a23[2])

        points = WorkcellPoints(
            spawn_a08=spawn_a08,
            start_a23=start_a23,
            end_a23=end_a23,
            start_a05=start_a05,
            end_a05=end_a05,
            trash_bin=trash_bin,
            human_zone=human_zone,
            ur5_zone=ur5_zone,
        )

        # Asset catalog + spawner
        catalog = AssetCatalog(asset_root=self.cfg.asset_root, names=self.cfg.assets)

        # Verify all expected asset files exist
        self._verify_assets(catalog)
        self._spawner = Spawner(prim_root=f"{self.cfg.prims.world}/Spawned", catalog=catalog)

        # Safety controller (slowdown when safety human approaches robots)
        self._safety = SafetyController(
            human_path=self.cfg.prims.human_safety,
            robot1_path=self.cfg.prims.ur10,
            robot2_path=self.cfg.prims.ur5e,
            radius_m=self.cfg.safety_radius_m,
        )

        # State machine
        self._sm = WorkcellStateMachine(
            points=points,
            spawner=self._spawner,
            catalog=catalog,
            conveyor_a08=conv_a08,
            conveyor_a23=conv_a23,
            conveyor_a05=conv_a05,
            defect_rate=self.cfg.defect_rate,
            stats=self.stats,
            t_pick_s=self.cfg.t_human_pick_s,
            t_check_s=self.cfg.t_human_check_s,
            t_screw_s=self.cfg.t_human_screw_s,
        )
        self._sm.set_target_units(self.cfg.target_units)

        log.info("Loaded workcell. Target units: %d, defect-rate=%.2f", self.cfg.target_units, self.cfg.defect_rate)

    def run_until_done(self) -> None:
        if self._world is None or self._sm is None:
            raise RuntimeError("Call load() first.")

        sim_seconds = 0.0

        while True:
            # World step
            self._world.step(render=True)
            dt = float(self.cfg.physics_dt)
            sim_seconds += dt
            self.stats.tick_time(dt)

            # Safety slowdown
            scale = self._safety.compute_speed_scale() if self._safety else 1.0
            self._apply_speed_scale(scale)

            # Update state machine (moves objects)
            self._sm.step(dt, set_pose_fn=self._set_pose_cached)

            if self._sm.is_done():
                break
            if sim_seconds >= self.cfg.max_sim_seconds:
                log.warning("Stopped due to max_sim_seconds=%.1f", self.cfg.max_sim_seconds)
                break

        self._print_report()

    # ---------------------- helpers ----------------------

    def _verify_assets(self, catalog: AssetCatalog) -> None:
        # Verify expected filenames exist (supports assets in subfolders).
        missing = catalog.missing_expected()
        if missing:
            lines = [f"- {k}: {v}" for k, v in missing.items()]
            raise FileNotFoundError(
                "Missing expected asset files under asset_root=\n"
                f"  {self.cfg.asset_root}\n\n"
                "These are required (field: filename):\n"
                + "\n".join(lines)
                + "\n\nFix: pass --asset-root pointing to your 'Belt Roller Support' folder (or any parent folder that contains the USDZs)."
            )


    def _apply_speed_scale(self, scale: float) -> None:
        """Apply speed scaling to all conveyors."""
        if self._sm is None:
            return
        base = self.cfg.conveyor_speed_mps
        slow = self.cfg.conveyor_slow_mps
        speed = max(slow, base * scale)
        self._sm.conv_a08.speed_mps = speed
        self._sm.conv_a23.speed_mps = speed
        self._sm.conv_a05.speed_mps = speed

    def _print_report(self) -> None:
        avg = self.stats.avg_cycle_s()
        hours_for_200 = self.stats.required_hours_for_units(200)
        log.info("---- REPORT ----")
        log.info("Completed units: %d", getattr(self._sm, "completed_units", 0))
        log.info("Recorded cycles: %d", len(self.stats.cycles_s))
        if avg > 0:
            log.info("Average cycle time: %.2f s (%.2f min)", avg, avg / 60.0)
            log.info("Estimated uptime for 200 units/day: %.2f hours/day", hours_for_200)
        else:
            log.info("No completed cycles recorded (avg cycle time = 0).")

    def _open_stage(self, usd_path: Path) -> None:
        from omni.isaac.core.utils.stage import open_stage  # type: ignore
        log.info("Opening stage: %s", usd_path)
        open_stage(str(usd_path))

    def _discover_conveyor_path(self, conveyor_prim: str) -> ConveyorPath:
        anchors = find_anchorpoints(conveyor_prim)
        if len(anchors) >= 2:
            p0 = get_world_pose(anchors[0]).position
            p1 = get_world_pose(anchors[-1]).position
            pts = [p0, p1]
            log.info("Conveyor %s anchorpoints: %s -> %s", conveyor_prim, anchors[0], anchors[-1])
        else:
            # fallback: use conveyor prim pose and create a short path
            c = get_world_pose(conveyor_prim).position
            pts = [c, (c[0] + 1.0, c[1], c[2])]
            log.warning("Could not find anchorpoints under %s; using fallback path.", conveyor_prim)
        return ConveyorPath(points=[tuple(float(x) for x in p) for p in pts])

    @staticmethod
    def _offset_z(p: Tuple[float, float, float], dz: float) -> Tuple[float, float, float]:
        return (float(p[0]), float(p[1]), float(p[2] + dz))

    def _set_pose_cached(self, prim_path: str, position_xyz: Tuple[float, float, float]) -> None:
        """Set world pose keeping orientation, caching quaternion for performance."""
        if prim_path not in self._pose_cache:
            q = get_world_pose(prim_path).orientation_xyzw
            self._pose_cache[prim_path] = q
        set_world_pose(prim_path, position_xyz, self._pose_cache[prim_path])
