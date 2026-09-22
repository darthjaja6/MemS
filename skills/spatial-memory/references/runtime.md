# Simulator setup

Use this guide to choose a simulator for a new setup. If the task already has a
working simulator or a physical robot connection, use that environment. Geometry
calculations and the [scene viewer](visualize.md) do not require a simulator.

## Inspect the machine

Run from the skill directory:

```bash
python scripts/detect_simulators.py
```

The script reports installed Isaac Sim and MuJoCo packages, standalone Isaac Sim
installations, the operating system, and NVIDIA GPU memory and driver information.
It reads installation metadata without launching either simulator.

For installations outside the detected locations, add
`--python /path/to/python` or `--isaac-root /path/to/isaacsim`. Each option can be
repeated. The result's `runtime_verified: false` means simulator startup has not
been checked.

## Choose a simulator

1. Prefer installed Isaac Sim if the machine meets that version's
   [system requirements](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html).
   Otherwise, use installed MuJoCo if it works on the machine.
2. If neither is usable, recommend installing Isaac Sim when the GPU and platform
   meet its requirements, or MuJoCo otherwise.
3. Start the selected simulator using its own launcher or Python environment to
   confirm it works, then reuse it for the session.

The detection script supplies information for this choice; it does not select or
install a simulator. `scene.py export` writes a static scene in MuJoCo's XML format
(MJCF) for inspection; it does not launch a simulation.
