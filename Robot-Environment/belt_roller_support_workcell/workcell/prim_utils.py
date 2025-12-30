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


def get_prim_children(prim_path: str) -> List[str]:
    """Return child prim paths (one level)."""
    _require_isaac()
    from omni.isaac.core.utils.prims import get_prim_children as _get  # type: ignore
    return list(_get(prim_path))


def prim_exists(prim_path: str) -> bool:
    _require_isaac()
    from omni.isaac.core.utils.prims import is_prim_path_valid  # type: ignore
    return bool(is_prim_path_valid(prim_path))


def get_world_pose(prim_path: str) -> Pose:
    _require_isaac()
    from omni.isaac.core.utils.prims import get_prim_world_pose  # type: ignore
    p, q = get_prim_world_pose(prim_path)
    # p, q are numpy arrays
    return Pose(tuple(float(x) for x in p), tuple(float(x) for x in q))


def set_world_pose(prim_path: str, position: Sequence[float], orientation_xyzw: Optional[Sequence[float]] = None) -> None:
    _require_isaac()
    from omni.isaac.core.utils.prims import set_prim_world_pose  # type: ignore
    if orientation_xyzw is None:
        pose = get_world_pose(prim_path)
        orientation_xyzw = pose.orientation_xyzw
    set_prim_world_pose(prim_path, position, orientation_xyzw)


def add_reference(usd_path: str, prim_path: str) -> None:
    _require_isaac()
    from omni.isaac.core.utils.stage import add_reference_to_stage  # type: ignore
    add_reference_to_stage(usd_path, prim_path)


def delete_prim(prim_path: str) -> None:
    _require_isaac()
    from omni.isaac.core.utils.prims import delete_prim as _del  # type: ignore
    _del(prim_path)


def ensure_xform(prim_path: str) -> None:
    """Ensure a prim exists as Xform. If not, create it."""
    _require_isaac()
    from omni.isaac.core.utils.prims import create_prim  # type: ignore
    if not prim_exists(prim_path):
        create_prim(prim_path, "Xform")


def find_anchorpoints(conveyor_prim: str) -> List[str]:
    """
    Find anchorpoint children under a conveyor prim. Your PDF shows nodes named
    Anchorpoint / Anchorpoint_01 etc.
    """
    if not prim_exists(conveyor_prim):
        log.warning("Conveyor prim does not exist: %s", conveyor_prim)
        return []
    kids = get_prim_children(conveyor_prim)
    anchors = [k for k in kids if k.split("/")[-1].lower().startswith("anchorpoint")]
    anchors.sort()
    return anchors
