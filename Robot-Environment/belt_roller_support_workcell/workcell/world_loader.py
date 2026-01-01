from __future__ import annotations

import logging
from pathlib import Path
from typing import Optional

# Allow running this file directly (Script Editor users sometimes hit Ctrl+Enter on it).
# When executed outside the package context, relative imports break.
try:
    from .config import WorkcellConfig
    from .assets import AssetCatalog
    from .prim_utils import (
        Pose,
        _require_isaac,
        find_anchorpoints,
        get_world_pose,
        set_world_pose,
    )
    from .controllers.spawner import Spawner
    from .controllers.conveyor import ConveyorController, ConveyorPath
    from .controllers.safety import SafetyController
    from .controllers.state_machine import WorkcellPoints, WorkcellStateMachine
    from .analytics.throughput import ThroughputStats
except Exception:  # pragma: no cover
    # Fallback for direct execution
    from workcell.config import WorkcellConfig
    from workcell.assets import AssetCatalog
    from workcell.prim_utils import (
        Pose,
        _require_isaac,
        find_anchorpoints,
        get_world_pose,
        set_world_pose,
    )
    from workcell.controllers.spawner import Spawner
    from workcell.controllers.conveyor import ConveyorController, ConveyorPath
    from workcell.controllers.safety import SafetyController
    from workcell.controllers.state_machine import WorkcellPoints, WorkcellStateMachine
    from workcell.analytics.throughput import ThroughputStats

log = logging.getLogger(__name__)


class WorkcellWorld:
    """Orchestrates:
    - stage loading
    - conveyor anchor discovery
    - controller wiring
    - simulation stepping loop
    """

    def __init__(self, cfg: WorkcellConfig):
        self.cfg = cfg
        self.stats = ThroughputStats()
        self._world = None
        self._spawner: Optional[Spawner] = None
        self._conveyor: Optional[ConveyorController] = None
        self._safety: Optional[SafetyController] = None
        self._sm: Optional[WorkcellStateMachine] = None

    # ----------------------------
    # Stage + physics setup
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
        """Ensure a PhysicsScene exists so World.step() has a physics context.

        If your USD (task.usd) has no PhysicsScene prim, Isaac's physics context can be None,
        causing: AttributeError: 'NoneType' object has no attribute 'get_physics_dt'
        """
        _require_isaac()
        try:
            from omni.isaac.core.utils.stage import get_current_stage  # type: ignore
            from pxr import UsdPhysics, PhysxSchema, Gf  # type: ignore
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

        # Author basic gravity; PhysX scene api is optional but safe.
        scene = UsdPhysics.Scene.Define(stage, scene_path)
        if not scene.GetGravityDirectionAttr():
            scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        if not scene.GetGravityMagnitudeAttr():
            scene.CreateGravityMagnitudeAttr().Set(9.81)

        try:
            physx_api = PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath(scene_path))
            # Some versions use different attr names; ignore if not present.
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

    # ----------------------------
    # Public API used by main.py
    # ----------------------------
    def load(self) -> None:
        """Load stage and build controllers."""
        env_path = str(self.cfg.env_usd_path)
        self._open_stage(env_path)
        self._ensure_physics_scene()

        # Create World after physics scene exists
        from omni.isaac.core import World  # type: ignore

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

        # Discover conveyor anchors
        conveyor_path = self.cfg.prims.conveyor_a08  # type: ignore[attr-defined]
        anchors = find_anchorpoints(conveyor_path)
        if not anchors:
            log.warning("No Anchorpoint children found under %s", conveyor_path)

        # Controllers
        self._spawner = Spawner(self.cfg, AssetCatalog(self.cfg.asset_root))
        self._conveyor = ConveyorController(self.cfg, anchors)
        self._safety = SafetyController(self.cfg)
        self._sm = WorkcellStateMachine(
            cfg=self.cfg,
            spawner=self._spawner,
            conveyor=self._conveyor,
            safety=self._safety,
            stats=self.stats,
        )

    def run_until_done(self) -> None:
        """Step simulation until the state machine reports completion."""
        if self._world is None or self._sm is None:
            raise RuntimeError("WorkcellWorld not loaded. Call load() first.")

        self._ensure_timeline_playing()

        max_seconds = float(getattr(self.cfg, "max_sim_seconds", 60.0))
        physics_dt = float(getattr(self.cfg, "physics_dt", 1.0 / 60.0))
        max_steps = int(max_seconds / physics_dt) if physics_dt > 0 else int(max_seconds * 60)

        for step_i in range(max_steps):
            # Update logic
            self._sm.step()

            # Stop if state machine says done
            if getattr(self._sm, "done", False):
                break

            # Step physics/render
            try:
                self._world.step(render=True)
            except AttributeError as e:
                # Typical symptom: missing PhysicsScene -> no physics context
                raise RuntimeError(
                    "World.step() failed due to missing physics context. "
                    "Ensure your USD has a PhysicsScene or keep _ensure_physics_scene() enabled."
                ) from e

        log.info("Simulation finished (steps=%d).", step_i + 1)
