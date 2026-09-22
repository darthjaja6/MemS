"""Inventory simulator installations and GPUs without starting either engine."""

import argparse
from concurrent.futures import ThreadPoolExecutor
import csv
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys


PACKAGE_PROBE = """
import importlib.metadata as metadata, json
found = {}
for name in ('isaacsim', 'isaacsim-kernel', 'isaacsim-app', 'isaacsim-core', 'mujoco'):
    try:
        found[name] = metadata.version(name)
    except metadata.PackageNotFoundError:
        pass
print(json.dumps(found))
"""


def python_candidates(extra=()):
    home = Path.home()
    candidates = [
        Path(sys.executable),
        Path.cwd() / ".venv/bin/python",
        Path.cwd() / ".venv/Scripts/python.exe",
        *map(Path, extra),
    ]
    for variable in ("VIRTUAL_ENV", "CONDA_PREFIX"):
        if os.environ.get(variable):
            prefix = Path(os.environ[variable])
            candidates.extend([prefix / "bin/python", prefix / "python.exe"])
    for command in ("python", "python3"):
        if shutil.which(command):
            candidates.append(Path(shutil.which(command)))
    for prefix in (
        Path(sys.base_prefix),
        home / "miniconda3",
        home / "anaconda3",
        home / "miniforge3",
    ):
        for env in (prefix / "envs").glob("*"):
            candidates.extend([env / "bin/python", env / "python.exe"])
    return sorted({str(path.absolute()) for path in candidates if path.is_file()})


def probe_python(executable):
    try:
        result = subprocess.run(
            [executable, "-c", PACKAGE_PROBE],
            capture_output=True,
            text=True,
            timeout=10,
            check=True,
        )
        return {"python": executable, "packages": json.loads(result.stdout)}
    except (OSError, subprocess.SubprocessError, ValueError) as error:
        return {"python": executable, "error": str(error)}


def probe_gpu():
    executable = shutil.which("nvidia-smi")
    if executable is None:
        return {"devices": [], "status": "nvidia-smi unavailable"}
    try:
        result = subprocess.run(
            [
                executable,
                "--query-gpu=name,memory.total,driver_version",
                "--format=csv,noheader,nounits",
            ],
            capture_output=True,
            text=True,
            timeout=10,
            check=True,
        )
        devices = [
            {
                "name": row[0].strip(),
                "vram_mib": int(float(row[1])),
                "driver": row[2].strip(),
            }
            for row in csv.reader(result.stdout.splitlines())
            if len(row) == 3
        ]
        return {"devices": devices, "status": "detected"}
    except (OSError, subprocess.SubprocessError, ValueError) as error:
        return {"devices": [], "status": "query failed", "error": str(error)}


def isaac_roots(extra=()):
    home = Path.home()
    candidates = [
        home / "isaacsim",
        Path("/opt/isaacsim"),
        Path("/isaac-sim"),
        Path.cwd() / "_isaac_sim",
        *map(Path, extra),
    ]
    for variable in ("ISAACSIM_PATH", "ISAAC_SIM_PATH"):
        if os.environ.get(variable):
            candidates.append(Path(os.environ[variable]))
    candidates.extend((home / ".local/share/ov/pkg").glob("isaac*"))
    installations = []
    for root in sorted(set(candidates)):
        launchers = [
            root / name
            for name in ("isaac-sim.sh", "isaac-sim.bat")
            if (root / name).is_file()
        ]
        if launchers:
            installations.append(
                {
                    "root": str(root.absolute()),
                    "launcher": str(launchers[0].absolute()),
                    "version": (root / "VERSION").read_text().strip()
                    if (root / "VERSION").is_file()
                    else None,
                }
            )
    return installations


def inventory(pythons=(), roots=()):
    with ThreadPoolExecutor(max_workers=4) as pool:
        environments = list(pool.map(probe_python, python_candidates(pythons)))
    return {
        "system": {
            "os": platform.system(),
            "architecture": platform.machine(),
            "release": platform.freedesktop_os_release().get("PRETTY_NAME")
            if sys.platform.startswith("linux")
            else platform.release(),
        },
        "gpu": probe_gpu(),
        "python_environments": environments,
        "isaac_standalone": isaac_roots(roots),
        "runtime_verified": False,
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--python",
        action="append",
        default=[],
        help="Additional environment Python executable",
    )
    parser.add_argument(
        "--isaac-root",
        action="append",
        default=[],
        help="Additional standalone Isaac Sim directory",
    )
    args = parser.parse_args()
    print(json.dumps(inventory(args.python, args.isaac_root), indent=2))
