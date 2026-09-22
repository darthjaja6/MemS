# Visualize the Scene and Recorded Movements

Use the viewer to inspect the current `scene.json` and replay saved robot
movements. It displays the scene estimates and motion data that were saved;
it does not reconstruct missing observations. Viewing is optional and does not
control the robot.

## Open the viewer

From the `spatial_memory` package directory:

```bash
python demo/app.py --scene /path/to/scene.json --runs /path/to/runs
```

Open http://127.0.0.1:8765. Use `--port` to choose another port. `--runs` is
the directory containing recorded sessions; the viewer searches its subdirectories.
With no arguments, `python demo/app.py` opens the bundled recorded stacking example.

## Live scene

The live view shows the latest saved scene. Changes to `scene.json` and files
under its `assets/` directory refresh the display. Object positions, orientations,
scales, mirror transforms and colors come from those files. Objects without
drawable geometry remain listed in the sidebar.
Support surfaces use neutral shading to keep objects distinct. Select an object
in the scene or sidebar to inspect its dimensions, position, and source; use
**Focus object** to move the camera closer.

Robot shapes come from the URDF, the file describing the robot's links and
joints. The viewer calculates each link's position from the saved joint values.
The display is as current as those saved values, rather than a direct camera feed.

The left sidebar lists objects followed by recorded sessions. Select a session
to play or seek through it, change playback speed, and view the paths traced by
the gripper or other mounted device. Each tool has a distinct path color; the
solid section is elapsed motion, the faint section is later in the recording,
and a dot marks the current tool position. A checkpoint is a pause for the agent to
review observations; its marker jumps to that saved time. **Current scene**
returns to the latest scene. If a file cannot be
read, the previous display stays visible and the connection status changes.

## Replay files

For a recording in the viewer's JSON format, place `replay.json` in a session
directory under `--runs`. The structure is:

```json
{
  "title": "Motion sequence",
  "kind": "recorded",
  "scene_path": "/path/to/original/scene.json",
  "scene": {"assets": {}, "objects": {}, "robots": {}},
  "frames": [
    {"time": 0, "robots": {}, "objects": {}}
  ],
  "actions": [],
  "checkpoints": []
}
```

- `title` is the session name shown in the sidebar.
- `kind` is `recorded` for execution data or `preview` for simulated motion.
  Previews are labeled **Preview**. Replaying either kind leaves the live scene unchanged.
- `scene` is the initial scene snapshot. Replace the empty dictionaries above
  with the actual assets, objects and robot models for the recording.
- `scene_path` sets the location used to resolve relative asset and URDF paths.
  Use an absolute path to the original scene file. If omitted, paths resolve
  beside `replay.json`.
- `frames` contains samples in time order. Each sample's `time` is elapsed
  seconds. `robots[name].joints` maps joint names to values in the robot model's
  units (radians for rotating joints, metres for sliding joints).
  `objects[name]` contains a world `position` in metres and an orientation
  `quaternion`, a four-number rotation representation in `[w, x, y, z]` order.
  Include complete object poses at each
  sample so seeking does not depend on earlier playback.
- Optional `actions` entries contain `name`, `start` and `end`; optional
  `checkpoints` entries contain `name` and `time`. All times are elapsed seconds.
- Optional `scenes` entries contain `time` and a complete `scene` snapshot, so
  geometry changes can be replayed as well as poses. Samples may include `tips`
  (tool names mapped to world XYZ coordinates in metres) to show saved tool paths
  without loading robot geometry. `trajectory_source` describes how they were obtained.

## RoboDojo archives

RoboDojo task workspaces can be replayed directly from their `scene.json`,
`observations/NNN/observation.json`, `bridge/actions_NNN.json` and
`scene_NNN.json` files under `--runs`.

Between observations, the animation uses commanded joint values. At saved
observations, it shows measured joint values. The display labels these as
**Commanded joint motion** and **Measured checkpoint**, respectively. Thus the
intermediate animation shows what was commanded, not proof of what happened.

Objects change only when a saved scene snapshot supplies an update. Archives
without scene snapshots show robot motion without object models. Timing uses
`control_period_s`, the seconds per control step stored in the scene, with
0.04 seconds as the fallback for historical RoboDojo logs.

## Older SO-101 recordings

Older SO-101 recordings use Markdown tables for scene state and `events.jsonl`
for measured joints. To open that format:

```bash
python demo/app.py --memory /path/to/memory.md --runs /path/to/runs
```

`--scene` takes precedence over `--memory`.

Markdown tables accept object columns `Object`, `x / m`, `y / m`, `z / m`,
`Geometry`, `Relation`; joint columns `Joint`, `Angle / rad`; and a plane row
`table`, `nx`, `ny`, `nz`, `d / m`. Geometry supports `box length width height`,
`ellipsoid rx ry rz`, and `cup outer_radius inner_radius height` in metres.
Coordinates use the robot base frame.

`events.jsonl` retains measured-joint replay and time spent reviewing observations.
`robot_mapping.json` in the session or runs directory supplies joint offsets and
gripper calibration. Historical object positions come from saved planning
estimates. Positions of objects held by the gripper are inferred from recorded
grasp confirmations and arm motion, rather than tracked independently in images.
