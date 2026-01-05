from __future__ import annotations

import logging
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

# Allow running this file directly (Script Editor users sometimes hit Ctrl+Enter on it).
# When executed outside the package context, relative imports break.
try:
    from .config import WorkcellConfig
    from .assets import AssetCatalog
    from .prim_utils import (
        _require_isaac,
        find_anchorpoints,
        get_world_pose,
        set_world_pose,
        prim_exists,
    )
    from .controllers.spawner import Spawner
    from .controllers.conveyor import ConveyorController, ConveyorPath
    from .controllers.safety import SafetyController
    from .controllers.state_machine import WorkcellPoints, WorkcellStateMachine
    from .analytics.throughput import ThroughputStats
except Exception:  # pragma: no cover
    from workcell.config import WorkcellConfig
    from workcell.assets import AssetCatalog
    from workcell.prim_utils import (
        _require_isaac,
        find_anchorpoints,
        get_world_pose,
        set_world_pose,
        prim_exists,
    )
    from workcell.controllers.spawner import Spawner
    from workcell.controllers.conveyor import ConveyorController, ConveyorPath
    from workcell.controllers.safety import SafetyController
    from workcell.controllers.state_machine import WorkcellPoints, WorkcellStateMachine
    from workcell.analytics.throughput import ThroughputStats

log = logging.getLogger(__name__)


def _lift(p: Sequence[float], dz: float = 0.02) -> Tuple[float, float, float]:
    return (float(p[0]), float(p[1]), float(p[2]) + float(dz))


class WorkcellWorld:
    """Orchestrates:
    - stage loading
    - conveyor path discovery
    - controller wiring
    - simulation stepping loop
    """

    def __init__(self, cfg: WorkcellConfig):
        self.cfg = cfg
        self.stats = ThroughputStats()
        self._world = None

        # Wired during load()
        self._catalog: Optional[AssetCatalog] = None
        self._spawner: Optional[Spawner] = None
        self._sm: Optional[WorkcellStateMachine] = None
        self._safety: Optional[SafetyController] = None
        self._conv_a08: Optional[ConveyorController] = None
        self._conv_a23: Optional[ConveyorController] = None
        self._conv_a05: Optional[ConveyorController] = None

    # ----------------------------
    # Stage helpers
    # ----------------------------
    def _open_stage(self, usd_path: str) -> None:
        _require_isaac()
        try:
            from omni.isaac.core.utils.stage import open_stage  # type: ignore

            ok = open_stage(str(usd_path))
            if ok is False:
                raise RuntimeError(f"open_stage returned False for: {usd_path}")
        except Exception as e:
            raise RuntimeError(f"Failed to open USD stage: {usd_path}") from e

    def _ensure_physics_scene(self) -> None:
        """Ensure a PhysicsScene exists so World.step() has a physics context."""
        _require_isaac()
        try:
            from omni.isaac.core.utils.stage import get_current_stage  # type: ignore
            from pxr import UsdPhysics, PhysxSchema  # type: ignore
        except Exception:
            return

        stage = get_current_stage()

        # If any PhysicsScene already exists, keep it.
        for prim in stage.Traverse():
            tn = prim.GetTypeName()
            if tn in ("PhysicsScene", "PhysxScene"):
                return

        scene_path = "/World/physicsScene"
        if not stage.GetPrimAtPath(scene_path).IsValid():
            stage.DefinePrim(scene_path, "PhysicsScene")

        try:
            UsdPhysics.SceneAPI.Apply(stage.GetPrimAtPath(scene_path))
        except Exception:
            pass

        # Optionally enable CCD if available
        try:
            physx_api = PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath(scene_path))
            if hasattr(physx_api, "CreateEnableCCDAttr"):
                physx_api.CreateEnableCCDAttr().Set(True)
        except Exception:
            pass

    def _ensure_timeline_playing(self) -> None:
        """World.step() is happiest when the timeline is playing."""
        _require_isaac()
        try:
            import omni.timeline  # type: ignore

            tl = omni.timeline.get_timeline_interface()
            if tl and not tl.is_playing():
                tl.play()
        except Exception:
            pass

    def _infer_conveyor_points(self, conveyor_prim: str) -> List[Tuple[float, float, float]]:
        """Build a polyline path for a conveyor.

        Prefers Anchorpoint children (Anchorpoint*, sorted). If none exist, falls back to a
        two-point path inferred from the prim's world bounding box, or as a last resort,
        a short segment along +X.
        """
        pts: List[Tuple[float, float, float]] = []

        # 1) Anchorpoints (recommended)
        anchors = find_anchorpoints(conveyor_prim)
        for a in anchors:
            try:
                pts.append(tuple(get_world_pose(a).position))
            except Exception:
                continue
        if len(pts) >= 2:
            return pts

        # 2) Bounding box fallback
        try:
            from omni.isaac.core.utils.stage import get_current_stage  # type: ignore
            from pxr import UsdGeom, Usd  # type: ignore

            stage = get_current_stage()
            prim = stage.GetPrimAtPath(conveyor_prim)
            if prim and prim.IsValid():
                cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), includedPurposes=[UsdGeom.Tokens.default_])
                bbox = cache.ComputeWorldBound(prim)
                r = bbox.GetRange()
                mn = r.GetMin()
                mx = r.GetMax()
                cx, cy, cz = (mn[0] + mx[0]) / 2.0, (mn[1] + mx[1]) / 2.0, (mn[2] + mx[2]) / 2.0
                ext = (abs(mx[0] - mn[0]), abs(mx[1] - mn[1]), abs(mx[2] - mn[2]))
                axis = int(max(range(3), key=lambda i: ext[i]))
                if axis == 0:
                    return [(float(mn[0]), float(cy), float(cz)), (float(mx[0]), float(cy), float(cz))]
                if axis == 1:
                    return [(float(cx), float(mn[1]), float(cz)), (float(cx), float(mx[1]), float(cz))]
        except Exception:
            pass

        # 3) Last resort: use conveyor prim pose + small offset
        try:
            p = get_world_pose(conveyor_prim).position
            return [tuple(p), (float(p[0]) + 1.0, float(p[1]), float(p[2]))]
        except Exception:
            return [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)]

    # ----------------------------
    # Public API used by main.py
    # ----------------------------
    def load(self) -> None:
        """Load stage and build controllers."""
        self.cfg.validate()

        env_path = str(self.cfg.env_usd_path)
        self._open_stage(env_path)
        self._ensure_physics_scene()

        # Create World after physics scene exists
        try:
            from omni.isaac.core import World  # type: ignore
        except Exception:  # pragma: no cover
            from isaacsim.core.api.world import World  # type: ignore

        stage_units = float(getattr(self.cfg, "stage_units_in_meters", 1.0) or 1.0)
        physics_dt = float(getattr(self.cfg, "physics_dt", 1.0 / 60.0))
        render_dt = float(getattr(self.cfg, "render_dt", physics_dt))

        self._world = World(
            stage_units_in_meters=stage_units,
            physics_dt=physics_dt,
            rendering_dt=render_dt,
        )

        # Reset builds physics context; do it after scene creation
        self._world.reset()
        if hasattr(self._world, "initialize_physics"):
            try:
                self._world.initialize_physics()
            except Exception:
                pass

        # Asset catalog + spawner
        self._catalog = AssetCatalog(self.cfg.asset_root, self.cfg.assets)
        spawn_root = f"{self.cfg.prims.world.rstrip('/')}/Spawned"
        self._spawner = Spawner(spawn_root, self._catalog)

        # Conveyor paths
        pts_a08 = self._infer_conveyor_points(self.cfg.prims.conveyor_a08)
        pts_a23 = self._infer_conveyor_points(self.cfg.prims.conveyor_a23)
        pts_a05 = self._infer_conveyor_points(self.cfg.prims.conveyor_a05)

        path_a08 = ConveyorPath(points=pts_a08)
        path_a23 = ConveyorPath(points=pts_a23)
        path_a05 = ConveyorPath(points=pts_a05)

        self._conv_a08 = ConveyorController("A08", path_a08, speed_mps=float(self.cfg.conveyor_speed_mps))
        self._conv_a23 = ConveyorController("A23", path_a23, speed_mps=float(self.cfg.conveyor_speed_mps))
        self._conv_a05 = ConveyorController("A05", path_a05, speed_mps=float(self.cfg.conveyor_speed_mps))

        # Safety (slows conveyors when a human approaches either robot base)
        self._safety = SafetyController(
            human_path=self.cfg.prims.human_safety,
            robot1_path=self.cfg.prims.ur10,
            robot2_path=self.cfg.prims.ur5e,
            radius_m=float(self.cfg.safety_radius_m),
        )

        # Workcell key points
        spawn_a08 = _lift(pts_a08[0])
        start_a23 = _lift(pts_a23[0])
        end_a23 = _lift(pts_a23[-1])
        start_a05 = _lift(pts_a05[0])
        end_a05 = _lift(pts_a05[-1])

        # zones (best-effort; will still run if prims are missing)
        try:
            trash = tuple(get_world_pose(self.cfg.prims.trash_bin).position)
        except Exception:
            trash = (spawn_a08[0] + 1.0, spawn_a08[1], spawn_a08[2])

        try:
            human_zone = tuple(get_world_pose(self.cfg.prims.human_assemble).position)
        except Exception:
            human_zone = (end_a23[0] + 0.3, end_a23[1], end_a23[2])

        # assemble near end of A23 by default
        ur5_zone = (end_a23[0] + 0.15, end_a23[1], end_a23[2])

        points = WorkcellPoints(
            spawn_a08=spawn_a08,
            start_a23=start_a23,
            end_a23=end_a23,
            start_a05=start_a05,
            end_a05=end_a05,
            trash_bin=trash,
            human_zone=human_zone,
            ur5_zone=ur5_zone,
        )

        # State machine
        self._sm = WorkcellStateMachine(
            points=points,
            spawner=self._spawner,
            catalog=self._catalog,
            conveyor_a08=self._conv_a08,
            conveyor_a23=self._conv_a23,
            conveyor_a05=self._conv_a05,
            defect_rate=float(self.cfg.defect_rate),
            stats=self.stats,
            t_pick_s=float(self.cfg.t_human_pick_s),
            t_check_s=float(self.cfg.t_human_check_s),
            t_screw_s=float(self.cfg.t_human_screw_s),
        )
        self._sm.set_target_units(int(self.cfg.target_units))

        log.info(
            "Workcell loaded: targets=%d defect_rate=%.2f dt=%.4f",
            int(self.cfg.target_units),
            float(self.cfg.defect_rate),
            physics_dt,
        )

    def run_until_done(self) -> None:
        """Step simulation until the state machine reports completion."""
        if self._world is None or self._sm is None:
            raise RuntimeError("WorkcellWorld not loaded. Call load() first.")

        self._ensure_timeline_playing()

        max_seconds = float(getattr(self.cfg, "max_sim_seconds", 60.0))
        dt = float(getattr(self.cfg, "physics_dt", 1.0 / 60.0))
        max_steps = int(max_seconds / dt) if dt > 0 else int(max_seconds * 60)

        # set_pose_fn must preserve orientation
        def set_pose_fn(prim_path: str, pos_xyz: Sequence[float]) -> None:
            try:
                pose = get_world_pose(prim_path)
                set_world_pose(prim_path, pos_xyz, pose.orientation_xyzw)
            except Exception:
                set_world_pose(prim_path, pos_xyz, (0, 0, 0, 1))

        base_speed = float(self.cfg.conveyor_speed_mps)
        min_speed = float(getattr(self.cfg, "conveyor_slow_mps", 0.0) or 0.0)

        for step_i in range(max_steps):
            # Update timebase for analytics
            self.stats.tick_time(dt)

            # Safety slowdown
            scale = 1.0
            if self._safety is not None:
                try:
                    scale = float(self._safety.compute_speed_scale())
                except Exception:
                    scale = 1.0

            speed = max(min_speed, base_speed * scale)
            if self._conv_a08 is not None:
                self._conv_a08.speed_mps = speed
            if self._conv_a23 is not None:
                self._conv_a23.speed_mps = speed
            if self._conv_a05 is not None:
                self._conv_a05.speed_mps = speed

            # Update logic + conveyors
            self._sm.step(dt, set_pose_fn=set_pose_fn)

            # Step physics/render
            try:
                self._world.step(render=True)
            except AttributeError as e:
                raise RuntimeError(
                    "World.step() failed due to missing physics context. "
                    "Ensure your USD has a PhysicsScene or keep _ensure_physics_scene() enabled."
                ) from e

            # Stop if completed
            if self._sm.is_done():
                break

        log.info(
            "Simulation finished: units=%d avg_cycle=%.2fs required_hours@200/day=%.2f",
            getattr(self._sm, "completed_units", 0),
            self.stats.avg_cycle_s(),
            self.stats.required_hours_for_units(200),
        )
