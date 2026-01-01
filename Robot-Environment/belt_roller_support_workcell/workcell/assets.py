from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Dict, Optional

from .config import AssetNames


class PartType(str, Enum):
    BASE = "base"
    BRACKET = "bracket"
    BUSH = "bush"
    ROLLER = "roller"
    SHAFT = "shaft"
    SCREW = "screw"


class Quality(str, Enum):
    OK = "ok"
    DEFECT = "defect"


@dataclass(frozen=True)
class AssetSpec:
    name: str
    usd_path: Path
    scale: float = 1.0


@dataclass
class AssetCatalog:
    asset_root: Path
    names: AssetNames

    def __post_init__(self) -> None:
        # Build a filename -> path index (recursive) so your assets can be kept in subfolders.
        self._index: Dict[str, Path] = {}
        self._index_lower: Dict[str, Path] = {}
        exts = {".usdz", ".usd", ".usda", ".usdc"}
        for p in self.asset_root.rglob("*"):
            try:
                if p.is_file() and p.suffix.lower() in exts:
                    self._index[p.name] = p.resolve()
                    self._index_lower[p.name.lower()] = p.resolve()
            except Exception:
                continue

    def _p(self, filename: str) -> Path:
        # Exact match first
        if filename in self._index:
            return self._index[filename]
        # Case-insensitive fallback (Windows filesystems are usually case-insensitive)
        low = filename.lower()
        if low in self._index_lower:
            return self._index_lower[low]
        # Final fallback: old behavior (flat folder)
        p = (self.asset_root / filename).resolve()
        if p.exists():
            return p
        raise FileNotFoundError(
            f"Asset '{filename}' not found under asset_root={self.asset_root}. "
            "Tip: pass --asset-root to the parent folder that contains your asset subfolders."
        )

    def expected_filenames(self) -> Dict[str, str]:
        return {k: str(v) for k, v in vars(self.names).items()}

    def missing_expected(self) -> Dict[str, str]:
        """Return mapping field->filename for missing expected assets."""
        missing: Dict[str, str] = {}
        for field, fn in self.expected_filenames().items():
            try:
                _ = self._p(fn)
            except Exception:
                missing[field] = fn
        return missing

    def get_part(self, part: PartType, quality: Quality) -> AssetSpec:
        # Use defect versions only where available; else fall back to OK asset.
        if part == PartType.BASE:
            fn = self.names.base_def if quality == Quality.DEFECT else self.names.base
        elif part == PartType.BRACKET:
            fn = self.names.bracket_def if quality == Quality.DEFECT else self.names.bracket
        elif part == PartType.BUSH:
            fn = self.names.bush_def if quality == Quality.DEFECT else self.names.bush
        elif part == PartType.SHAFT:
            fn = self.names.shaft_def if quality == Quality.DEFECT else self.names.shaft
        elif part == PartType.ROLLER:
            fn = self.names.roller
        elif part == PartType.SCREW:
            fn = self.names.screw
        else:
            raise ValueError(f"Unknown part: {part}")

        return AssetSpec(name=fn, usd_path=self._p(fn))

    def assembled_step(self, step_idx: int) -> AssetSpec:
        if step_idx == 1:
            fn = self.names.as1
        elif step_idx == 2:
            fn = self.names.as2
        elif step_idx == 3:
            fn = self.names.as3
        elif step_idx == 4:
            fn = self.names.as4
        elif step_idx == 5:
            fn = self.names.final_product
        else:
            raise ValueError("assembled step must be 1..5")
        return AssetSpec(name=fn, usd_path=self._p(fn))
