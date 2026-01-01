# Isaac Sim + VS Code (Windows) Setup Guide (Fixing `import omni` / `workcell` imports)

This guide explains:
- Why VS Code shows **“Import 'omni' could not be resolved”**
- How to fix IntelliSense/autocomplete using `python.analysis.extraPaths`
- How to **run your simulation correctly** (inside Isaac Sim Python)
- Common errors and the exact fixes

---

## 1) Your Isaac Sim install path (confirmed)

From your screenshots, Isaac Sim is installed here:

**`C:\Isaac-sim\isaac-sim-standalone-5.1.0-windows-x86_64`**

This folder contains `python.bat`, which is the **only** Python environment that can import `omni.*`.

---

## 2) Why VS Code says `import omni` cannot be resolved

### ✅ What it means
VS Code (Pylance) is analyzing your code using your **system Python** or your project `.venv`,
but `omni` is **not a normal pip package**.

`omni` exists only inside Isaac Sim’s extension folders and is available at runtime when you run:
- Isaac Sim GUI (Script Editor), OR
- Isaac Sim’s `python.bat`

### ✅ Key point
Even if you “fix” VS Code warnings, the script will still fail if you run it using normal Python.

---

## 3) Fix VS Code IntelliSense (recommended)

Edit this file:

**`C:\Users\ardiw\AppData\Roaming\Code\User\settings.json`**

Add (or merge) this:

```jsonc
{
  "python.analysis.extraPaths": [
    "C:/Isaac-sim/isaac-sim-standalone-5.1.0-windows-x86_64/exts",
    "C:/Isaac-sim/isaac-sim-standalone-5.1.0-windows-x86_64/extscache",
    "C:/Isaac-sim/isaac-sim-standalone-5.1.0-windows-x86_64/kit/exts",
    "C:/Isaac-sim/isaac-sim-standalone-5.1.0-windows-x86_64/kit/extsPhysics",
    "C:/Isaac-sim/isaac-sim-standalone-5.1.0-windows-x86_64/kit/extsUser",
    "C:/Isaac-sim/isaac-sim-standalone-5.1.0-windows-x86_64/kit/kernel/py"
  ],

  "python.analysis.diagnosticSeverityOverrides": {
    "reportMissingImports": "none"
  }
}
