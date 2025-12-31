from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence, Tuple

log = logging.getLogger(__name__)


@dataclass
class Pose:
    position: Tuple[float, float, float]
    orientation_xyzw: Tuple[float, float, float, float]


def _require_isaac() -> None:
    try:
        import omni  # noqa: F401
    except Exception as e:
        raise ImportError("Isaac Sim Python APIs not available. Run inside Isaac Sim.") from e


def prim_exists(prim_path: str) -> bool:
    _require_isaac()
    from omni.isaac.core.utils.stage import get_current_stage  # type: ignore

    stage = get_current_stage()
    prim = stage.GetPrimAtPath(prim_path)
    return prim.IsValid()


def get_prim_children(prim_path: str) -> List[str]:
    _require_isaac()
    from omni.isaac.core.utils.stage import get_current_stage  # type: ignore

    stage = get_current_stage()
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return []
    return [c.GetPath().pathString for c in prim.GetChildren()]


def find_anchorpoints(conveyor_prim: str) -> List[str]:
    if not prim_exists(conveyor_prim):
        log.warning("Conveyor prim does not exist: %s", conveyor_prim)
        return []
    kids = get_prim_children(conveyor_prim)
    anchors = [k for k in kids if k.split("/")[-1].lower().startswith("anchorpoint")]
    anchors.sort()
    return anchors