from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

log = logging.getLogger(__name__)


def _dist(a: Sequence[float], b: Sequence[float]) -> float:
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)


def _lerp(a: Sequence[float], b: Sequence[float], t: float) -> Tuple[float, float, float]:
    return (a[0] + (b[0]-a[0])*t, a[1] + (b[1]-a[1])*t, a[2] + (b[2]-a[2])*t)


@dataclass
class ConveyorPath:
    """Polyline path. Provide at least 2 points in world coordinates."""
    points: List[Tuple[float, float, float]]

    seg_lengths: List[float] = field(init=False)
    total_length: float = field(init=False)

    def __post_init__(self):
        if len(self.points) < 2:
            raise ValueError("ConveyorPath needs >= 2 points")
        self.seg_lengths = []
        for i in range(len(self.points)-1):
            self.seg_lengths.append(_dist(self.points[i], self.points[i+1]))
        self.total_length = sum(self.seg_lengths)

    def sample(self, s: float) -> Tuple[float, float, float]:
        """Sample at distance s along the path."""
        if s <= 0:
            return self.points[0]
        if s >= self.total_length:
            return self.points[-1]

        acc = 0.0
        for i, seg_len in enumerate(self.seg_lengths):
            if acc + seg_len >= s:
                t = (s - acc) / max(seg_len, 1e-9)
                return _lerp(self.points[i], self.points[i+1], t)
            acc += seg_len
        return self.points[-1]


@dataclass
class ConveyorItem:
    prim_path: str
    s: float = 0.0  # distance along conveyor
    done: bool = False


class ConveyorController:
    """
    Kinematic conveyor: moves registered prims along a ConveyorPath by updating their world pose.

    This is intentionally simple and does not rely on any Isaac conveyor extension.
    """
from omni.isaac.core.utils.prims import get_prim_at_path
from pxr import UsdPhysics, Gf

class ConveyorController:
    def __init__(self, prim_path: str, speed: float = 0.3):
        self.prim_path = prim_path
        self.speed = speed
        self.prim = get_prim_at_path(prim_path)

        if not self.prim.IsValid():
            raise RuntimeError(f"Conveyor not found: {prim_path}")

        self.rb = UsdPhysics.RigidBodyAPI.Apply(self.prim)

    def update(self):
        # Move along +X (change if your conveyor axis differs)
        self.rb.GetVelocityAttr().Set(Gf.Vec3f(self.speed, 0.0, 0.0))

    def add_item(self, prim_path: str, start_s: float = 0.0) -> None:
        self.items[prim_path] = ConveyorItem(prim_path=prim_path, s=start_s, done=False)

    def remove_item(self, prim_path: str) -> None:
        self.items.pop(prim_path, None)

    def step(self, dt: float, set_pose_fn) -> None:
        """
        set_pose_fn(prim_path, position_xyz)
        """
        for item in list(self.items.values()):
            if item.done:
                continue
            item.s += self.speed_mps * dt
            pos = self.path.sample(item.s)
            set_pose_fn(item.prim_path, pos)

            if item.s >= self.path.total_length:
                item.done = True

    def is_done(self, prim_path: str) -> bool:
        it = self.items.get(prim_path)
        return bool(it and it.done)
