from __future__ import annotations

"""
workcell.prim_utils

Small USD/Isaac helper functions used across the workcell project.

Key design goals
- Be safe to import in VS Code (even if Pylance can't resolve `omni`).
- Work across Isaac Sim versions by minimizing reliance on version-sensitive helpers.
- Avoid APIs that sometimes expect a Usd.Prim vs a prim-path string (the source of
  "str object has no attribute GetChildren" errors).
"""

import logging
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple, TYPE_CHECKING

log = logging.getLogger(__name__)

if TYPE_CHECKING:  # for IDEs only (does not run at runtime)
    import omni  # noqa: F401


@dataclass(frozen=True)
class Pose:
    position: Tuple[float, float, float]
    # Quaternion order used in this project: (x, y, z, w)
    orientation_xyzw: Tuple[float, float, float, float]


# ---------------------------
# Isaac / Stage helpers
# ---------------------------

def _require_isaac() -> None:
    """Raise a clear error if this is executed outside Isaac Sim."""
    try:
        import omni  # noqa: F401
    except Exception as e:
        raise ImportError(
            "Isaac Sim Python APIs not available. "
            "Run this script using Isaac Sim (GUI Script Editor) or Isaac's python.bat."
        ) from e


def _get_stage():
    """Return the current USD stage (works across Isaac Sim versions)."""
    _require_isaac()

    # Preferred (Isaac Core)
    try:
        from omni.isaac.core.utils.stage import get_current_stage  # type: ignore
        stage = get_current_stage()
        if stage is not None:
            return stage
    except Exception:
        pass

    # Fallback (Omniverse USD context)
    try:
        import omni.usd  # type: ignore
        return omni.usd.get_context().get_stage()
    except Exception as e:
        raise RuntimeError("Could not obtain current USD stage from Isaac Sim.") from e


def _norm_asset_path(p: str) -> str:
    """USD generally prefers forward slashes even on Windows."""
    return p.replace("\\", "/")


# ---------------------------
# Basic prim operations
# ---------------------------

def prim_exists(prim_path: str) -> bool:
    stage = _get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    return bool(prim and prim.IsValid())


def ensure_xform(prim_path: str):
    """
    Ensure prim exists as an Xform at `prim_path` and return the Usd.Prim.
    Creates missing prims (and parents) as needed.
    """
    stage = _get_stage()
    from pxr import UsdGeom  # type: ignore

    prim = stage.GetPrimAtPath(prim_path)
    if prim and prim.IsValid():
        return prim

    x = UsdGeom.Xform.Define(stage, prim_path)
    return x.GetPrim()


def delete_prim(prim_path: str) -> None:
    """Delete a prim if it exists."""
    stage = _get_stage()

    # Try Isaac helper first (if present)
    try:
        from omni.isaac.core.utils.prims import delete_prim as _delete  # type: ignore
        _delete(prim_path)
        return
    except Exception:
        pass

    # USD fallback
    prim = stage.GetPrimAtPath(prim_path)
    if prim and prim.IsValid():
        stage.RemovePrim(prim_path)


def get_prim_children(prim_path: str) -> List[str]:
    """
    Return child prim paths (one level).

    IMPORTANT:
    We intentionally do NOT call `omni.isaac.core.utils.prims.get_prim_children`
    because some Isaac Sim versions expect a Usd.Prim there (and will crash or
    throw if you pass a string prim-path).
    """
    stage = _get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        return []
    return [c.GetPath().pathString for c in prim.GetChildren()]


def add_reference(prim_path: str, usd_path: str, *, clear_existing: bool = False) -> str:
    """
    Add a USD reference to `prim_path` pointing at `usd_path`.

    Returns:
        The prim path that received the reference (usually `prim_path`).
    """
    stage = _get_stage()
    prim = ensure_xform(prim_path)

    ref_path = _norm_asset_path(usd_path)
    if clear_existing:
        try:
            prim.GetReferences().ClearReferences()
        except Exception:
            pass
    prim.GetReferences().AddReference(ref_path)
    return prim.GetPath().pathString


# ---------------------------
# Pose helpers (world space)
# ---------------------------

def get_world_pose(prim_path: str) -> Optional[Pose]:
    """
    Read world pose of a prim.

    Returns:
        Pose in (position xyz, quaternion xyzw) or None if prim invalid.
    """
    stage = _get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        return None

    # Try Isaac helper first (fast and stable if available)
    try:
        from omni.isaac.core.utils.xforms import get_world_pose as _get  # type: ignore
        pos, quat = _get(prim_path)  # quat often wxyz
        pos_t = (float(pos[0]), float(pos[1]), float(pos[2]))
        q = tuple(float(x) for x in quat)
        if len(q) == 4:
            # Heuristic: many Isaac APIs return wxyz. Convert to xyzw.
            # If your version returns xyzw already, both represent same values but reordered.
            q_xyzw = (q[1], q[2], q[3], q[0])
            return Pose(pos_t, q_xyzw)
    except Exception:
        pass

    # USD fallback (works across versions)
    from pxr import UsdGeom  # type: ignore
    xcache = UsdGeom.XformCache(0)
    m = xcache.GetLocalToWorldTransform(prim)
    t = m.ExtractTranslation()
    q = m.ExtractRotationQuat()  # Gf.Quatd, (real=w, imaginary=xyz)
    qi = q.GetImaginary()
    pos_t = (float(t[0]), float(t[1]), float(t[2]))
    q_xyzw = (float(qi[0]), float(qi[1]), float(qi[2]), float(q.GetReal()))
    return Pose(pos_t, q_xyzw)


def set_world_pose(
    prim_path: str,
    position: Tuple[float, float, float],
    orientation_xyzw: Tuple[float, float, float, float],
) -> None:
    """Set world pose using Isaac helper if available, otherwise write USD ops."""
    stage = _get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        raise ValueError(f"Prim does not exist: {prim_path}")

    # Preferred: Isaac helper
    qx, qy, qz, qw = orientation_xyzw
    q_wxyz = (qw, qx, qy, qz)
    try:
        from omni.isaac.core.utils.xforms import set_world_pose as _set  # type: ignore
        try:
            _set(prim_path, position, q_wxyz)  # many versions expect wxyz
            return
        except Exception:
            _set(prim_path, position, orientation_xyzw)  # some versions accept xyzw
            return
    except Exception:
        pass

    # USD fallback: use Orient + Translate ops
    from pxr import UsdGeom, Gf  # type: ignore
    xf = UsdGeom.Xformable(prim)

    # Find or create ops
    translate_op = None
    orient_op = None
    for op in xf.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            translate_op = op
        elif op.GetOpType() == UsdGeom.XformOp.TypeOrient:
            orient_op = op

    if translate_op is None:
        translate_op = xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
    if orient_op is None:
        orient_op = xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)

    translate_op.Set(Gf.Vec3d(*[float(x) for x in position]))
    orient_op.Set(Gf.Quatd(float(qw), Gf.Vec3d(float(qx), float(qy), float(qz))))


# ---------------------------
# Workcell-specific helpers
# ---------------------------

def find_anchorpoints(conveyor_prim: str) -> List[str]:
    """
    Find anchorpoint children under a conveyor prim.
    Looks for nodes named Anchorpoint / Anchorpoint_01 etc.
    """
    if not prim_exists(conveyor_prim):
        log.warning("Conveyor prim does not exist: %s", conveyor_prim)
        return []

    kids = get_prim_children(conveyor_prim)
    anchors = [k for k in kids if k.split("/")[-1].lower().startswith("anchorpoint")]
    anchors.sort()
    return anchors
