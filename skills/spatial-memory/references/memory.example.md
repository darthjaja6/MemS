# Spatial environment

This is an empty template for the older SO-101 Markdown viewer. Current tasks
store their scene in [`scene.json`](scene.md); they do not need a separate
Markdown memory file. For opening either format, see [the viewer guide](visualize.md).

## Robot and tools

The viewer uses the bundled [SO-101 robot model](../assets/so101/robot.urdf).
Describe the mounted gripper and camera setup here if needed. This text is for
the reader; it does not change the robot model shown by the viewer.

## Workspace

Describe relevant surfaces and how their locations were measured.

## Objects

Enter observed object centres in metres, using the robot base as the coordinate
origin and the axes defined by its model. `Geometry` accepts one of these forms,
with all dimensions in metres:

- `box length width height`
- `ellipsoid rx ry rz`, where each value is a radius along the corresponding axis
- `cup outer_radius inner_radius height`

`Relation` is a text description, such as which surface supports the object.

| Object | x / m | y / m | z / m | Geometry | Relation |
| --- | --- | --- | --- | --- | --- |

## Current joint pose

Enter measured angles in radians for the joints named in the robot model.
Angles must use that model's zero position and positive rotation direction;
convert raw device readings with the robot's calibration before filling rows.

| Joint | Angle / rad |
| --- | --- |

## Current task

No task recorded.
