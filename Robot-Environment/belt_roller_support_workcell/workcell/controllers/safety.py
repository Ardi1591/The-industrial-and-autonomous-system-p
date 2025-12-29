from __future__ import annotations

import logging
import math
from dataclasses import dataclass
from typing import Sequence

from ..prim_utils import get_world_pose, prim_exists

log = logging.getLogger(__name__)


def _dist(a: Sequence[float], b: Sequence[float]) -> float:
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)


@dataclass
class SafetyController:
    """Simple proximity safety: if human is close to either robot base, slow conveyors."""
    human_path: str
    robot1_path: str
    robot2_path: str
    radius_m: float

    def compute_speed_scale(self) -> float:
        if not (prim_exists(self.human_path) and prim_exists(self.robot1_path) and prim_exists(self.robot2_path)):
            return 1.0

        h = get_world_pose(self.human_path).position
        r1 = get_world_pose(self.robot1_path).position
        r2 = get_world_pose(self.robot2_path).position
        d = min(_dist(h, r1), _dist(h, r2))
        if d < self.radius_m:
            # linear slowdown towards 0.4x at d=0
            scale = max(0.4, d / self.radius_m)
            return float(scale)
        return 1.0
