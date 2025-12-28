# Belt Roller Support Workcell (Isaac Sim / Python, OOP)

This project is an **OOP Python** implementation for the belt-roller-support workcell 
- Load the environment USD (`task.usd`)
- Spawn **all provided assets** (good + defective components, assembled-step assets, final product)
- Run a simple state-machine to move parts along conveyors, simulate UR10 sorting, and simulate UR5 + humans creating the assembled product (illusion via spawn/despawn)
- Measure cycle time and compute required uptime for **200 units/day**

> Designed to run **inside NVIDIA Isaac Sim on Windows** (run with Isaac Sim's `python.bat`).

## 1) Put assets somewhere on Windows

You can either:
- Copy your assets into a local folder, for example:
  `C:\workcell_assets\`
  containing:
  - `Base.usdz`, `Bracket.usdz`, `Bush.usdz`, `Roller.usdz`, `Shaft.usdz`, `Screw.usdz`
  - defect: `base-1.usdz`, `Bracket-1.usdz`, `BUSH-1.usdz`, `Shaft-1.usdz`
  - assembled steps: `Assemble (1).usdz` ... `Assemble (4).usdz`
  - final: `Belt Roller Support.usdz`

or keep your current OneDrive folder structure (with subfolders like
`Component`, `Defect Component`, `Assembled step`).

✅ This code now supports assets inside **subfolders** because it searches recursively.

## 2) Run (Windows / Isaac Sim)

> Note: if you click a `.bat` in Isaac Sim's **Content Browser** you will see:
> **"Unsupported extension bat"**. That's expected — `.bat` is not an asset.
> Run the command from Windows Terminal / PowerShell.

Open **Isaac Sim Command Prompt** (or a normal cmd/PowerShell with Isaac env), then:

```bat
cd C:\path\to\belt_roller_support_workcell
"C:\Isaac-sim\python.bat" main.py ^
  --env-usd "C:\Users\<you>\OneDrive\Documents\Fusion\Robot-Environment\task.usd" ^
  --asset-root "C:\Users\<you>\OneDrive\Documents\Fusion\Robot-Environment\Belt Roller Support" ^
  --units 10 ^
  --defect-rate 0.15
```

### Even easier: auto-detect

If `task.usd` and your `Belt Roller Support` folder are near the project, you can run:

```bat
"C:\Isaac-sim\python.bat" main.py
```

### Headless mode
```bat
"C:\Isaac-sim\python.bat" main.py --headless 1 --env-usd "..." --asset-root "..."
```

## 3) What you may need to adjust

This code **auto-discovers** conveyor endpoints by searching for child prims named like `Anchorpoint` under:
- `/World/ConveyorBelt_A08`
- `/World/ConveyorBelt_A23`
- `/World/ConveyorBelt_A05`

If your prim paths differ, change them in `workcell/config.py` (WorkcellPrimPaths).

Also: the logic is intentionally “illusion based” (spawn/despawn) because your PDF states UR5 assembly is too complex to replicate fully.

## 4) Output

At the end, it prints:
- number of completed units
- average cycle time
- estimated hours required to reach 200 units/day


