from __future__ import annotations

import logging
import random
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional, Sequence, Set, Tuple

from ..assets import AssetCatalog, PartType, Quality
from ..prim_utils import Pose, get_world_pose
from .spawner import Spawner, SpawnedObject
from .conveyor import ConveyorController
from ..analytics.throughput import ThroughputStats

log = logging.getLogger(__name__)


@dataclass(frozen=True)
class WorkcellPoints:
    """Important world points (meters)."""
    spawn_a08: Tuple[float, float, float]
    start_a23: Tuple[float, float, float]
    end_a23: Tuple[float, float, float]
    start_a05: Tuple[float, float, float]
    end_a05: Tuple[float, float, float]
    trash_bin: Tuple[float, float, float]
    human_zone: Tuple[float, float, float]
    ur5_zone: Tuple[float, float, float]


class State(str, Enum):
    IDLE = "idle"
    WAIT_A08_END = "wait_a08_end"
    WAIT_A23_END = "wait_a23_end"
    ASSEMBLE_AS3 = "assemble_as3"
    ASSEMBLE_AS4 = "assemble_as4"
    PLACE_FINAL = "place_final"
    WAIT_FINAL_END = "wait_final_end"
    SCRAP = "scrap"


@dataclass
class ActivePart:
    prim_path: str
    part: PartType
    quality: Quality


@dataclass
class UnitBuild:
    required: List[PartType]
    received: Set[PartType] = field(default_factory=set)
    start_time: Optional[float] = None


class WorkcellStateMachine:
    """
    High-level workcell behavior matching your PDF:
    - Spawn parts on conveyor A08.
    - UR10 "sorts": defective parts are not transferred to A23.
    - Good parts go to A23 and reach UR5 area.
    - UR5 + humans "assemble" using illusion: spawn AS3 -> AS4 -> final product.
    - Final product moves on A05 to end.
    """

    def __init__(
        self,
        points: WorkcellPoints,
        spawner: Spawner,
        catalog: AssetCatalog,
        conveyor_a08: ConveyorController,
        conveyor_a23: ConveyorController,
        conveyor_a05: ConveyorController,
        defect_rate: float,
        stats: ThroughputStats,
        t_pick_s: float,
        t_check_s: float,
        t_screw_s: float,
    ):
        self.points = points
        self.spawner = spawner
        self.catalog = catalog
        self.conv_a08 = conveyor_a08
        self.conv_a23 = conveyor_a23
        self.conv_a05 = conveyor_a05
        self.defect_rate = defect_rate
        self.stats = stats

        self.t_pick_s = t_pick_s
        self.t_check_s = t_check_s
        self.t_screw_s = t_screw_s

        self.state: State = State.IDLE
        self.time_in_state: float = 0.0

        self.unit = UnitBuild(required=[
            PartType.BASE,
            PartType.BRACKET,
            PartType.BUSH,
            PartType.SHAFT,
            PartType.ROLLER,
            PartType.SCREW,
        ])

        self._active_part: Optional[ActivePart] = None
        self._active_assembled: Optional[str] = None
        self._final_product_prim: Optional[str] = None

        self.completed_units: int = 0
        self.target_units: int = 1

    def set_target_units(self, n: int) -> None:
        self.target_units = max(1, int(n))

    def is_done(self) -> bool:
        return self.completed_units >= self.target_units

def _spawn_next_part(self) -> None:
    remaining = [p for p in self.unit.required if p not in self.unit.received]
    if not remaining:
        return

    part = remaining[0]
    quality = Quality.DEFECT if random.random() < self.defect_rate else Quality.OK

    asset = self.catalog.get_part(part, quality)

    obj = self.spawner.spawn(
        asset,
        position=self.points.spawn_a08,
        prefix=f"{part.value}_{quality.value}",   # <-- readable prim grouping
    )
    self._active_part = ActivePart(prim_path=obj.prim_path, part=part, quality=quality)

    self.conv_a08.add_item(obj.prim_path, start_s=0.0)

    if self.unit.start_time is None:
        self.unit.start_time = self.stats.sim_time_s

    log.info("Spawned part %s (%s)", part.value, quality.value)

    def _scrap_current_unit(self, reason: str) -> None:
        log.warning("Scrapping unit: %s", reason)

        # Illustrate "incomplete set to trash bin": spawn AS1 and AS2 near humans and delete shortly
        try:
            as1 = self.catalog.assembled_step(1)
            as2 = self.catalog.assembled_step(2)

            o1 = self.spawner.spawn(as1, position=self.points.human_zone)
            o2 = self.spawner.spawn(as2, position=(self.points.human_zone[0] + 0.2, self.points.human_zone[1], self.points.human_zone[2]))

            # immediately move them into bin area (illusion)
            self.stats.defer_delete(o1.prim_path, delay_s=1.0)
            self.stats.defer_delete(o2.prim_path, delay_s=1.0)
        except Exception as e:
            log.debug("Scrap-illustration failed: %s", e)

        # reset unit
        self.unit = UnitBuild(required=list(self.unit.required))
        self._active_part = None
        self._active_assembled = None
        self._final_product_prim = None
        self.state = State.IDLE
        self.time_in_state = 0.0

    def step(self, dt: float, set_pose_fn) -> None:
        """
        Called every simulation step.
        `set_pose_fn(prim_path, pos_xyz)` should update a prim pose (orientation kept).
        """
        self.time_in_state += dt

        # Move items along conveyors
        self.conv_a08.step(dt, set_pose_fn=set_pose_fn)
        self.conv_a23.step(dt, set_pose_fn=set_pose_fn)
        self.conv_a05.step(dt, set_pose_fn=set_pose_fn)

        # Let the stats system delete deferred prims (for scrap illusion)
        self.stats.step_deferred_deletes(dt)

        if self.is_done():
            return

        if self.state == State.IDLE:
            self._spawn_next_part()
            self.state = State.WAIT_A08_END
            self.time_in_state = 0.0
            return

        if self.state == State.WAIT_A08_END:
            if not self._active_part:
                self.state = State.IDLE
                self.time_in_state = 0.0
                return

            if self.conv_a08.is_done(self._active_part.prim_path):
                # at the end of A08: UR10 decides
                self.conv_a08.remove_item(self._active_part.prim_path)

                if self._active_part.quality == Quality.DEFECT:
                    # Not picked by UR10 -> scrap this unit
                    self.spawner.despawn(self._active_part.prim_path)
                    self._scrap_current_unit(f"Defective part not picked by UR10: {self._active_part.part.value}")
                    return

                # Good: transfer to A23 (teleport to start A23 and continue)
                set_pose_fn(self._active_part.prim_path, self.points.start_a23)
                self.conv_a23.add_item(self._active_part.prim_path, start_s=0.0)
                self.state = State.WAIT_A23_END
                self.time_in_state = 0.0
            return

        if self.state == State.WAIT_A23_END:
            if not self._active_part:
                self.state = State.IDLE
                self.time_in_state = 0.0
                return

            if self.conv_a23.is_done(self._active_part.prim_path):
                self.conv_a23.remove_item(self._active_part.prim_path)

                # UR5 "takes" the part -> despawn part and mark received
                self.spawner.despawn(self._active_part.prim_path)
                self.unit.received.add(self._active_part.part)
                log.info("UR5 received part: %s (%d/%d)", self._active_part.part.value, len(self.unit.received), len(self.unit.required))
                self._active_part = None

                if len(self.unit.received) >= len(self.unit.required):
                    # all parts received -> assemble illusion
                    self.state = State.ASSEMBLE_AS3
                    self.time_in_state = 0.0

                    # spawn AS3 in front of UR5 (illusion)
                    as3 = self.catalog.assembled_step(3)
                    obj = self.spawner.spawn(as3, position=self.points.ur5_zone)
                    self._active_assembled = obj.prim_path
                else:
                    self.state = State.IDLE
                    self.time_in_state = 0.0
            return

        if self.state == State.ASSEMBLE_AS3:
            if self.time_in_state >= self.t_pick_s:
                # AS3 -> AS4
                if self._active_assembled:
                    self.spawner.despawn(self._active_assembled)
                as4 = self.catalog.assembled_step(4)
                obj = self.spawner.spawn(as4, position=self.points.ur5_zone)
                self._active_assembled = obj.prim_path

                self.state = State.ASSEMBLE_AS4
                self.time_in_state = 0.0
            return

        if self.state == State.ASSEMBLE_AS4:
            if self.time_in_state >= (self.t_check_s + self.t_screw_s):
                # AS4 -> final product on conveyor A05
                if self._active_assembled:
                    self.spawner.despawn(self._active_assembled)
                    self._active_assembled = None

                final_asset = self.catalog.assembled_step(5)
                obj = self.spawner.spawn(final_asset, position=self.points.start_a05)
                self._final_product_prim = obj.prim_path
                self.conv_a05.add_item(obj.prim_path, start_s=0.0)

                self.state = State.WAIT_FINAL_END
                self.time_in_state = 0.0
            return

        if self.state == State.WAIT_FINAL_END:
            if self._final_product_prim and self.conv_a05.is_done(self._final_product_prim):
                self.conv_a05.remove_item(self._final_product_prim)
                self.spawner.despawn(self._final_product_prim)

                # Record stats
                start = self.unit.start_time if self.unit.start_time is not None else self.stats.sim_time_s
                end = self.stats.sim_time_s
                self.stats.record_cycle(start_s=start, end_s=end)

                self.completed_units += 1
                log.info("✅ Completed unit %d / %d (cycle=%.2fs)", self.completed_units, self.target_units, end - start)

                # reset for next unit
                self.unit = UnitBuild(required=list(self.unit.required))
                self._final_product_prim = None
                self.state = State.IDLE
                self.time_in_state = 0.0
            return
