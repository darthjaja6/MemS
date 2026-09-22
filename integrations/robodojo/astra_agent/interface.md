# RoboDojo command interface

## Prepared inputs

- `observation.json`: task instruction, RGB paths, measured state and budget.
- `scene.json`: robot geometry, tool offsets, calibrated cameras, assets and instances.
- `runtime.json`: selected simulator and interpreter.

The integration refreshes measured robot/camera state after execution. Camera
calibration comes from the active rendered camera; object ground truth and
evaluator predicates are outside the task inputs.

## Decisions

Write `decision.json`, then run `python robot.py decision.json`.
In command examples, replace `python` with the absolute interpreter in `runtime.json`.
The same decision can contain:

| Field | Content |
| --- | --- |
| `scene_update` | Fields to merge into the authoritative scene; null removes a field |
| `attachments` | Observed relationships `{object,robot}`; null robot detaches |
| `actions` | Ordered tool targets, gripper commands and a final review checkpoint |
| `contacts` | Intentional contact pairs: object/surface names or robot tool names |
| `finish` | `{status:complete|unable, reason:...}` based on observations |

A scene-only decision publishes without moving.

Each action has `left` and/or `right` tool fields, `steps` and optional `checkpoint`.
Omitted fields retain previous commands. A tool accepts:

| Field | Meaning |
| --- | --- |
| `target` | World point, object name, or `{object,landmark?,bounds?,offset?}`; see the skill's `references/scene.md` |
| `approach`, `closing` | Required perpendicular world axes at the target |
| `pose` | Full controlled-link pose `[x,y,z,qw,qx,qy,qz]` |
| `gripper` | 0 closed, 1 open |

A position-only target keeps previous orientation as a preference. Supplied axes
or full poses constrain the endpoint. Orientation may vary along the route.
The tool geometry converts contact points to controlled-link poses automatically.
`home:true` requests both initial arm poses and openings; initial state is in the scene.
RoboDojo completion includes returning both arms to their initial poses. A
finish decision appends a 48-step home motion unless its final
action already requests home or the environment is terminal. Reserve those steps
in planning; the execution interface reserves them in the episode budget. Both complete and unable finishes
use this same completion procedure.

Each action requests 1–64 steps at 25 Hz; a sequence totals at most 96 steps.
Motion starts from measured joints. Unchanged gripper commands allow continuous
passage through waypoints. Put the review checkpoint at the sequence end.
`contacts` permits only named pairs in geometric planning; a robot name means
its tool links.

Attachment transforms predict object motion between observations.
A final action can accompany a finish decision.
Other runs, benchmark task/reward source and internal object state are outside
this workspace's task inputs.
