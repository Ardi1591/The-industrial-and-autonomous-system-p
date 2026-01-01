from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Tuple


@dataclass(frozen=True)
class WorkcellPrimPaths:
    """Prim paths in the provided `task.usd` environment."""
    world: str = "/World"

    # Conveyors in your PDF screenshots
    conveyor_a08: str = "/World/ConveyorBelt_A08"
    conveyor_a23: str = "/World/ConveyorBelt_A23"
    conveyor_a05: str = "/World/ConveyorBelt_A05"

    # Robots
    ur10: str = "/World/ur10_short_suction"
    ur5e: str = "/World/ur5e"

    # Humans
    human_qc: str = "/World/male_adult_construction_02"  # supervisor/QC near bin
    human_assemble: str = "/World/male_adult_construction_03"  # tightening/screwing
    human_safety: str = "/World/male_adult_construction_01"  # comes near robots

    # Bin (small_KLT_visual_collision in PDF)
    trash_bin: str = "/World/small_KLT_visual_collision"

    # Camera sensor in PDF (used conceptually)
    camera_sensor: str = "/World/Camera_SG2_OX03CC_5200_GMSL2_H60YA"


@dataclass(frozen=True)
class AssetNames:
    """Expected filenames (as provided)."""
    # good components
    base: str = "Base.usdz"
    bracket: str = "Bracket.usdz"
    bush: str = "Bush.usdz"
    roller: str = "Roller.usdz"
    shaft: str = "Shaft.usdz"
    screw: str = "Screw.usdz"

    # defective components
    base_def: str = "base-1.usdz"
    bracket_def: str = "Bracket-1.usdz"
    bush_def: str = "BUSH-1.usdz"
    shaft_def: str = "Shaft-1.usdz"

    # assembled steps (illusion)
    as1: str = "Assemble (1).usdz"
    as2: str = "Assemble (2).usdz"
    as3: str = "Assemble (3).usdz"
    as4: str = "Assemble (4).usdz"
    final_product: str = "Belt Roller Support.usdz"


@dataclass
class WorkcellConfig:
    env_usd_path: Path
    asset_root: Path

    prims: WorkcellPrimPaths = field(default_factory=WorkcellPrimPaths)
    assets: AssetNames = field(default_factory=AssetNames)

    # production target / spawn ratios
    target_units: int = 10
    defect_rate: float = 0.15

    # simulation time
    physics_dt: float = 1.0 / 60.0
    render_dt: float = 1.0 / 60.0
    max_sim_seconds: float = 600.0

    # speed controls (meters/second for kinematic motion)
    conveyor_speed_mps: float = 0.25
    conveyor_slow_mps: float = 0.10  # slowed when human approaches

    # safety
    safety_radius_m: float = 1.0

    # Timings for illusion steps (seconds)
    t_human_pick_s: float = 2.0
    t_human_check_s: float = 2.5
    t_human_screw_s: float = 3.0

    def validate(self) -> None:
        if not self.env_usd_path.exists():
            raise FileNotFoundError(f"Environment USD not found: {self.env_usd_path}")
        if not self.asset_root.exists():
            raise FileNotFoundError(f"Asset root not found: {self.asset_root}")
        if not (0.0 <= self.defect_rate <= 1.0):
            raise ValueError("defect_rate must be in [0,1]")
