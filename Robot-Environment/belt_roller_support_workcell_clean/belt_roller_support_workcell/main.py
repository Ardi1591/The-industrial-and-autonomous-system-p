"""belt_roller_support_workcell.main

Entry point (Windows / Isaac Sim).

IMPORTANT:
  - Run this with Isaac Sim's Python (python.bat), NOT your system Python.
  - In Isaac Sim's *Content Browser*, a .bat file will show "Unsupported extension bat".
    That's normal — .bat is not an asset. Run it from Windows Terminal / PowerShell.

Example (PowerShell):

  cd C:\Users\<you>\OneDrive\Documents\Fusion\Robot-Environment\belt_roller_support_workcell
  C:/Isaac-sim/python.bat main.py

If you keep task.usd and the asset folders next to this script, you do NOT need to pass
--env-usd / --asset-root: they will be auto-detected.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from workcell.logging_utils import setup_logging
from workcell.sim_app import IsaacSimApp
from workcell.config import WorkcellConfig
from workcell.world_loader import WorkcellWorld


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser()
    p.add_argument(
        "--env-usd",
        type=str,
        default=None,
        help="Path to task.usd (optional). If omitted, the script auto-detects task.usd.",
    )
    p.add_argument(
        "--asset-root",
        type=str,
        default=None,
        help=(
            "Asset root folder (optional). Can be a flat folder of USDZs OR a parent folder that "
            "contains subfolders like 'Component', 'Defect Component', 'Assembled step'. "
            "If omitted, the script auto-detects a nearby folder with USDZ assets."
        ),
    )
    p.add_argument("--units", type=int, default=10, help="How many finished units to simulate.")
    p.add_argument("--defect-rate", type=float, default=0.15, help="Probability [0..1] to spawn defective part.")
    p.add_argument("--headless", type=int, default=0, help="1=headless, 0=GUI.")
    p.add_argument("--physics-dt", type=float, default=1.0 / 60.0, help="Physics timestep.")
    p.add_argument("--render-dt", type=float, default=1.0 / 60.0, help="Render timestep.")
    p.add_argument("--max-sim-seconds", type=float, default=600.0, help="Safety stop (seconds).")
    return p.parse_args()


def _running_inside_isaac_python() -> bool:
    """Best-effort check to warn users when they run with system Python."""
    try:
        import omni.kit.app  # type: ignore

        return True
    except Exception:
        try:
            import isaacsim  # type: ignore

            return True
        except Exception:
            return False


def _autodetect_env_usd(start: Path) -> Path:
    """Find task.usd near the project or current directory."""
    candidates: list[Path] = []

    def add(p: Path) -> None:
        p = p.resolve()
        if p not in candidates:
            candidates.append(p)

    add(start)
    add(Path.cwd())
    add(Path(__file__).resolve().parent)
    # Walk up a few parents so "Robot-Environment/.../belt_roller_support_workcell" works out-of-the-box
    for base in [start, Path.cwd(), Path(__file__).resolve().parent]:
        for i in range(1, 6):
            try:
                add(base.parents[i - 1])
            except Exception:
                break

    # Fast path: exact filename in common locations
    for base in candidates:
        p = base / "task.usd"
        if p.exists():
            return p.resolve()

    # Fallback: find any .usd file named task.usd in shallow recursion
    for base in candidates:
        try:
            for p in base.rglob("task.usd"):
                return p.resolve()
        except Exception:
            continue

    raise FileNotFoundError(
        "Could not auto-detect task.usd. Place task.usd next to this project or pass --env-usd <path-to-task.usd>."
    )


def run(
    env_usd: str | Path | None = None,
    asset_root: str | Path | None = None,
    *,
    units: int = 10,
    defect_rate: float = 0.15,
    headless: bool = False,
    physics_dt: float = 1.0 / 60.0,
    render_dt: float = 1.0 / 60.0,
    max_sim_seconds: float = 600.0,
) -> None:
    """Programmatic entry point.

    Useful if you run it from Isaac Sim's Script Editor.
    """
    setup_logging()
    if not _running_inside_isaac_python():
        raise RuntimeError("Run() must be called inside Isaac Sim's Python environment.")

    env_path = Path(env_usd).expanduser() if env_usd else _autodetect_env_usd(Path.cwd())
    asset_path = Path(asset_root).expanduser() if asset_root else _autodetect_asset_root(Path.cwd())

    cfg = WorkcellConfig(
        env_usd_path=env_path,
        asset_root=asset_path,
        target_units=int(units),
        defect_rate=float(defect_rate),
        physics_dt=float(physics_dt),
        render_dt=float(render_dt),
        max_sim_seconds=float(max_sim_seconds),
    )

    with IsaacSimApp(headless=bool(headless)):
        world = WorkcellWorld(cfg)
        world.load()
        world.run_until_done()


def _autodetect_asset_root(start: Path) -> Path:
    """Find a folder that contains (or contains subfolders with) .usdz assets."""
    bases: list[Path] = []
    def add(p: Path) -> None:
        p = p.resolve()
        if p not in bases:
            bases.append(p)

    add(start)
    add(Path.cwd())
    add(Path(__file__).resolve().parent)
    for base in [start, Path.cwd(), Path(__file__).resolve().parent]:
        for i in range(1, 6):
            try:
                add(base.parents[i - 1])
            except Exception:
                break

    # Prefer a folder literally named like your dataset
    preferred_names = [
        "Belt Roller Support",
        "Belt_Roller_Support",
        "assets",
        "Assets",
    ]
    for base in bases:
        for name in preferred_names:
            p = base / name
            if p.exists() and p.is_dir():
                # If it contains any usdz under it, accept
                if any(x.suffix.lower() == ".usdz" for x in p.rglob("*.usdz")):
                    return p.resolve()

    # Otherwise: find nearest folder that has at least 3 USDZ files
    for base in bases:
        if base.exists() and base.is_dir():
            usdz = list(base.rglob("*.usdz"))
            if len(usdz) >= 3:
                return base.resolve()

    raise FileNotFoundError(
        "Could not auto-detect an asset root folder containing .usdz files. "
        "Pass --asset-root pointing to your 'Belt Roller Support' folder (or a folder containing the USDZs)."
    )


def main() -> int:
    args = parse_args()

    try:
        run(
            env_usd=args.env_usd,
            asset_root=args.asset_root,
            units=args.units,
            defect_rate=args.defect_rate,
            headless=bool(args.headless),
            physics_dt=args.physics_dt,
            render_dt=args.render_dt,
            max_sim_seconds=args.max_sim_seconds,
        )
        return 0
    except Exception as e:
        # Ensure a clear message in Windows terminals
        print(f"ERROR: {e}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
