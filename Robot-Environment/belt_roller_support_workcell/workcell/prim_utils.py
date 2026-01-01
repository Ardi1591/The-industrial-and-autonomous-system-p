from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence, Tuple, Union

log = logging.getLogger(__name__)


@dataclass
class Pose:
    position: Tuple[float, float, float]
    orientation_xyzw: Tuple[float, float, float, float]


def _require_isaac() -> None:
    """Raise if not running inside Isaac Sim's Python."""
    try:
        import omni  # noqa: F401
    except Exception as e:
        raise ImportError(
            "Isaac Sim Python APIs not available. "
            "Run this using Isaac Sim (Script Editor) or Isaac's python.bat."
        ) from e


def _get_stage():
    _require_isaac()
    # Isaac has changed helper location/names across releases; try a few.
    try:
        from omni.isaac.core.utils.stage import get_current_stage  # type: ignore
        return get_current_stage()
    except Exception:
        pass
    try:
        from omni.isaac.core.utils.stage import get_current_stage as get_stage  # type: ignore
        return get_stage()
    except Exception as e:
        raise RuntimeError("Could not access USD stage from Isaac Sim.") from e


def prim_exists(prim_path: str) -> bool:
    """Return True if a prim exists at path."""
    stage = _get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    return bool(prim.IsValid())


def get_prim_children(prim: Union[str, object]) -> List[str]:
    """Return child prim paths (one level).

    Version-safe:
    - Some Isaac versions provide omni helper that accepts a prim path (str)
    - Some versions expect a Usd.Prim object (passing str can fail)

    This helper tries Isaac helper first, then falls back to querying USD.
    """
    _require_isaac()

    # Try Isaac's helper first
    try:
        from omni.isaac.core.utils.prims import get_prim_children as _get  # type: ignore
        children = list(_get(prim))  # may return strings or Usd.Prim
        if not children:
            return []
        if isinstance(children[0], str):
            return [str(c) for c in children]
        if hasattr(children[0], "GetPath"):
            return [c.GetPath().pathString for c in children]  # type: ignore
    except Exception:
        pass

    # Fallback: query stage directly
    stage = _get_stage()
    usd_prim = prim if hasattr(prim, "IsValid") else stage.GetPrimAtPath(str(prim))
    if not usd_prim or not usd_prim.IsValid():
        return []
    return [c.GetPath().pathString for c in usd_prim.GetChildren()]


def find_anchorpoints(conveyor_prim: str) -> List[str]:
    """Find children whose names start with 'Anchorpoint' under a conveyor prim."""
    if not prim_exists(conveyor_prim):
        log.warning("Conveyor prim does not exist: %s", conveyor_prim)
        return []
    kids = get_prim_children(conveyor_prim)
    anchors = [k for k in kids if k.split("/")[-1].lower().startswith("anchorpoint")]
    anchors.sort()
    return anchors


# ----------------------------
# References / prim management
# ----------------------------

def ensure_xform(prim_path: str) -> None:
    """Ensure an Xform prim exists at prim_path."""
    stage = _get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    if prim.IsValid():
        return
    stage.DefinePrim(prim_path, "Xform")


def delete_prim(prim_path: str) -> None:
    """Delete prim at prim_path (if it exists)."""
    stage = _get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return
    stage.RemovePrim(prim_path)


def add_reference(dst_prim_path: str, usd_path: str) -> None:
    """Create an Xform prim at dst_prim_path and add a USD reference to usd_path."""
    _require_isaac()
    stage = _get_stage()
    prim = stage.GetPrimAtPath(dst_prim_path)
    if not prim.IsValid():
        prim = stage.DefinePrim(dst_prim_path, "Xform")

    # Add a reference (works for .usd/.usda/.usdc/.usdz)
    try:
        prim.GetReferences().AddReference(str(usd_path))
    except Exception as e:
        raise RuntimeError(f"Failed to add reference: {usd_path} -> {dst_prim_path}") from e


# ----------------------------
# Pose helpers (world space)
# ----------------------------

def get_world_pose(prim_path: str) -> Pose:
    """Get world pose (position + quaternion xyzw) for a prim."""
    _require_isaac()
    # Prefer Isaac helpers when available
    try:
        from omni.isaac.core.utils.prims import get_prim_world_pose  # type: ignore
        p, q = get_prim_world_pose(prim_path)
        return Pose(tuple(float(x) for x in p), tuple(float(x) for x in q))
    except Exception:
        pass

    # Fallback: compute using USD Xformable
    stage = _get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        raise ValueError(f"Prim does not exist: {prim_path}")

    from pxr import Usd, UsdGeom  # type: ignore

    xform = UsdGeom.Xformable(prim)
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    mat = cache.GetLocalToWorldTransform(prim)
    trans = mat.ExtractTranslation()
    rot = mat.ExtractRotationQuat()
    # rot is Gf.Quatd(w, (x,y,z)); return xyzw
    imag = rot.GetImaginary()
    return Pose(
        (float(trans[0]), float(trans[1]), float(trans[2])),
        (float(imag[0]), float(imag[1]), float(imag[2]), float(rot.GetReal())),
    )
def set_world_pose(prim_path: str, pose: Pose) -> None:
    """Set world pose for a prim."""
    _require_isaac()
    try:
        from omni.isaac.core.utils.prims import set_prim_world_pose  # type: ignore
        set_prim_world_pose(prim_path, pose.position, pose.orientation_xyzw)
        return
    except Exception:
        pass

    # Fallback: author xform ops (local pose). This is less ideal but avoids crashing.
    stage = _get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        raise ValueError(f"Prim does not exist: {prim_path}")

    from pxr import UsdGeom, Gf  # type: ignore
    xform = UsdGeom.Xformable(prim)

    # Clear existing ops and write translate + orient (quaternion)
    xform.ClearXformOpOrder()
    t_op = xform.AddTranslateOp()
    t_op.Set(Gf.Vec3d(*pose.position))
    # Isaac uses xyzw; USD's Quatd is w + vec
    qx, qy, qz, qw = pose.orientation_xyzw
    o_op = xform.AddOrientOp()
    o_op.Set(Gf.Quatd(qw, Gf.Vec3d(qx, qy, qz)))


__all__ = [
    "Pose",
    "_require_isaac",
    "prim_exists",
    "get_prim_children",
    "find_anchorpoints",
    "ensure_xform",
    "delete_prim",
    "add_reference",
    "get_world_pose",
    "set_world_pose",
]
