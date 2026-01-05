from __future__ import annotations

import itertools
import logging
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence

from ..assets import AssetCatalog, AssetSpec
from ..prim_utils import add_reference, ensure_xform, set_world_pose, delete_prim

log = logging.getLogger(__name__)


@dataclass
class SpawnedObject:
    prim_path: str
    asset: AssetSpec


class Spawner:
    """
    Spawns USD/USDZ assets into the stage by creating a prim and referencing the asset.

    Key detail:
    - USD prim names must be "identifier-like" (no spaces, parentheses, etc.).
    - Your asset filenames include spaces like "Assemble (3).usdz", so we MUST sanitize.
    """

    _invalid = re.compile(r"[^0-9a-zA-Z_]+")

    def __init__(self, prim_root: str, catalog: AssetCatalog):
        self._root = prim_root.rstrip("/")
        self._catalog = catalog
        self._counter = itertools.count(1)
        ensure_xform(self._root)

    @classmethod
    def _sanitize_prim_name(cls, raw: str) -> str:
        """
        Convert any filename / label into a USD-friendly prim name.
        Example: "Assemble (3).usdz" -> "Assemble_3"
        """
        # remove extension if given a filename
        raw = Path(raw).stem

        # common cleanup
        raw = raw.replace("-", "_").replace(" ", "_")
        raw = raw.replace("(", "_").replace(")", "_")

        # strict cleanup
        raw = cls._invalid.sub("_", raw).strip("_")

        if not raw:
            raw = "asset"
        if raw[0].isdigit():
            raw = f"p_{raw}"
        return raw

    def spawn(
        self,
        asset: AssetSpec,
        position: Sequence[float],
        orientation_xyzw=(0, 0, 0, 1),
        prim_name: Optional[str] = None,
        prefix: Optional[str] = None,
    ) -> SpawnedObject:
        """
        prim_name: Optional explicit base name (will be sanitized).
        prefix: Optional prefix like "part", "AS3", etc. (will be sanitized).
        """
        idx = next(self._counter)

        base = prim_name if prim_name else asset.name  # use catalog filename by default
        base = self._sanitize_prim_name(base)

        if prefix:
            base = f"{self._sanitize_prim_name(prefix)}_{base}"

        prim_path = f"{self._root}/{base}_{idx:04d}"
        ensure_xform(prim_path)

        usd_ref = str(asset.usd_path).replace('\\', '/')
        add_reference(prim_path, usd_ref)
        set_world_pose(prim_path, position, orientation_xyzw)

        log.debug("Spawned %s from %s at %s", prim_path, asset.usd_path, position)
        return SpawnedObject(prim_path=prim_path, asset=asset)

    def despawn(self, prim_path: str) -> None:
        try:
            delete_prim(prim_path)
        except Exception as e:
            log.warning("Failed to delete prim %s: %s", prim_path, e)
