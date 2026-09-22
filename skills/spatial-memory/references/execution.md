# Plan and execute robot movements

Use this guide after choosing the next actions from the task and `scene.json`.
The agent specifies where the arm should move and what its gripper or other
end-mounted device should do. The prepared robot command calculates and executes
the joint movements, then returns observations for the next decision.

## Connect your own robot

The repository supplies geometry and planning libraries. A deployment developer
provides the command that connects them to a particular robot. The
[annotated scene](example_scene.json) describes the fields; replace its mock data
and remove its comments before loading it.

| Adapter responsibility | Required inputs or behavior |
| --- | --- |
| Describe the robot | URDF with local mesh paths, controlled `tip` link, ordered active `joint_names`, world `base_transform`, and `tool.point_in_tip`. Add `local_approach` and `local_closing` for direction-based scene targets. |
| Publish measured state | Read every movable joint, including passive/gripper joints, in radians or metres. Save `joints` and the measured or forward-kinematic world tip `pose`; refresh these after execution. |
| Publish observations | Save images and matching `K`, camera-to-world `T_base_camera`, and optional OpenCV `distortion`. Moving cameras need calibration for each observation; see [camera coordinates](geometry.md#coordinates-and-units). |
| Configure planning | Supply controller period, joint velocity/acceleration limits, collision obstacles, attachments, and any permitted contact pairs. Scene objects do not automatically become obstacles in the shared planner. |
| Execute and report | Map planned joint samples to controller channels and units, command them at the configured period, implement gripper operations and stopping, and return fresh observations at checkpoints. |

The Python entry points are `JointPlanner` and `ToolGoal` in
[`scripts/motion_planning/`](../scripts/motion_planning/). Construct the planner
with `robots`, a cache directory, `period_s`, `velocity`, `acceleration`, and
optional MPlib FCL `obstacles`. Models must be fixed-base with bounded revolute
or prismatic joints. Continuous, floating, and planar joints are unsupported.

Its call sequence is:

1. `set_state(joints)` with measured values in `planner.names` order. Names are
   prefixed as `robot_id_joint_name`; active joints precede passive joints.
2. `route({robot_id: ToolGoal(position, quaternion)})` with a world contact-point
   target and WXYZ tip orientation. Orientation is a preference unless
   `fixed_orientation=True`; omitted robots hold their measured joints.
3. `timed_path(route, minimum_steps)` to get samples starting at the measured
   state. Columns follow `planner.names[:planner.dof]`; rows are separated by
   `period_s`. The adapter sends these samples to its controller.

For a concrete implementation, read the RoboDojo adapter's
[`motion.py`](../../../integrations/robodojo/astra_agent/motion.py), which
converts scene meshes into obstacles and maps measured state and actions to
these calls. The [planner tests](../../../tests/test_motion_planning.py)
include a small synthetic robot example without a simulator or controller.
The separate [`execution.py`](../scripts/execution.py) offers optional sequence
and checkpoint handling for controllers implementing `read`, `move`, and `hold`;
it is not a robot driver. Give the task agent your deployment's action-command
reference once this connection is implemented.

## Describe the next sequence

An action can specify a target position, any required orientation, and operations
such as opening or closing the gripper. A **waypoint** is an intermediate target
along the movement; reaching one does not necessarily stop the robot.

A **checkpoint** ends a sequence so the agent can inspect new observations and
decide what to do next. Place one before a later action depends on an uncertain
earlier outcome. A pause needed for the mechanism to settle is different: it
can be part of a sequence without requesting an agent decision.

Use [scene target calculations](scene.md#tool-pose-and-ik) when you need to
convert a point and approach directions into the position and orientation of
the robot part being commanded. This accounts for the distance between that
part and the point on the gripper intended to reach the target. Robot commands
that accept scene targets perform this conversion themselves.

## Submit actions and read the result

Use the action format and command supplied with the robot workspace. The command
connects the shared planning scripts to that robot's movement and camera controls.

For the **RoboDojo robot benchmark**, the task workspace provides `robot.py`.
Write the actions to `decision.json` and run:

```bash
python robot.py decision.json
```

Replace `python` with the interpreter path in `runtime.json`. The workspace
includes `references/robot-interface.md` inside its copied skill. Read that
file for action fields, units, sequence limits and task completion commands.
It also explains how to include observed scene changes with the same decision.
Put the checkpoint on the last action of the submitted sequence.

The command returns the location of fresh observations, or an error explaining
why the sequence could not execute. Read the observation and its camera images,
check the actual result, and update affected objects through the
[scene commands](scene.md). In RoboDojo, measured robot and camera state is
refreshed automatically; object changes still need interpretation from images.

## How targets become joint movements

The prepared command reads the current measured joints, solves for joint angles
that reach each target, and plans a connected path from that starting state.
The planner uses robot joint limits and the scene geometry supplied to it.
It assigns timing to the path using joint speed and acceleration limits, then
sends the resulting joint commands to the robot.

The shared `motion_planning/` library performs these calculations. The separate
`execution.py` library runs sequences for robot setups that use it. Task agents
invoke their workspace's prepared command; they do not need to connect these
libraries themselves.

In RoboDojo, unchanged gripper commands allow continuous movement through
successive targets. The planner checks the reconstructed objects and objects
recorded as attached to the gripper. The decision's `contacts` field names pairs
that are allowed to touch; see the workspace command reference for its format.
These checks depend on the geometry recorded in `scene.json`.

Required directions and full orientations constrain the target, not the entire
route: orientation can change between targets. The shared planner checks target
position within 1 mm per coordinate and required directions within 1 degree.
These are numerical planning tolerances, not guarantees of physical accuracy.
