# Scene commands

Use these commands to locate the main surface, calculate object positions and
sizes from camera observations, and update `scene.json` as the robot acts. The
agent selects the object and measurements; each command calculates the requested
values and, where specified below, saves them to that object or surface.

## Command setup and coordinates

Run from the task workspace. These examples assume the skill is installed at
`.agents/skills/spatial-memory`:

For a new workspace, initialize an empty scene once:

```bash
printf '%s\n' '{}' > scene.json
```

Then add objects, cameras, and robots through `update`, or register shapes with
[`assets.py create`](modeling.md#create-and-place-an-asset):

```bash
python .agents/skills/spatial-memory/scripts/scene.py show
python .agents/skills/spatial-memory/scripts/scene.py update --input update.json
python .agents/skills/spatial-memory/scripts/scene.py export --output scene.xml
```

Commands read `scene.json` by default; `--scene path` selects another scene.
`--input path` supplies JSON arguments. Results print as JSON; `--output path`
saves them instead. The viewer and planner read `scene.json` directly. Use
`export` when you need a separate MJCF geometry snapshot; updates do not export one.

All world positions use metres in the scene's common coordinate frame. A **pose**
combines a position and an orientation. A quaternion represents orientation with
four numbers in WXYZ order; `[1,0,0,0]` means no rotation. Local
coordinates are relative to an object or robot link (a rigid segment of the
robot), as specified by the field. An asset is a reusable shape description;
a landmark is a named point on that shape.
The robot's observation-reading code supplies current robot and camera data.
Camera `K` maps camera-relative coordinates to pixels; `T_base_camera` maps
camera-relative coordinates into the scene's common frame. Camera axes are x
right, y down, z forward; pixels are `[u,v]` (column, row). A camera's optional
`distortion` holds OpenCV lens-distortion coefficients, such as `[k1,k2,p1,p2,k3]`.
Localization, projection, and fitting use these coefficients. For images that
are already undistorted, supply their adjusted `K` and omit `distortion`.

## Measure a plane

The main surface provides a reference for locating objects and choosing movement
heights. Its plane is stored under `surfaces` as `normal` and `offset_m`, satisfying
`normal · [x,y,z] + offset_m = 0`. The normal is a unit vector perpendicular to the
surface; the offset places the plane relative to the scene origin.

Choose measurements from the available camera and calibration data:

- **Multiple calibrated views:** use [`locate`](#locate-and-project) to calculate
  several surface points from matching image pixels. The camera positions must
  differ. This calculation does not require a known plane.
- **One camera without depth:** obtain surface points independently, for example
  by placing a calibrated point on the gripper on the surface and reading its
  position. A pixel alone cannot determine an unknown plane. Once the plane is
  measured, it can support single-view localization.

Save each measured point with:

```bash
python .agents/skills/spatial-memory/scripts/scene.py plane --input measurement.json
```

```json
{"name":"table","point":[0.1,-0.2,0.75]}
```

Alternatively, `{"name":"table","robot":"left"}` records the robot's configured
contact point at its current measured pose. Optional `link` and `offset` select a
particular robot link and a point in that link's coordinates. This command records
a measurement; it does not move the robot into contact.

Samples accumulate in `plane_samples.table`. After at least three non-collinear
points, call the same command with `{"name":"table","fit":true}`. It calculates
the best-fitting plane and saves `surfaces.table`. A point measurement can also
include `"fit":true` to calculate the plane immediately after adding that point.

If the plane parameters are already known, save them directly with
[`update`](#state-updates). A surface's optional `object` field identifies the
finite solid it belongs to; the plane is a reference for measurements, while that
object describes the surface's physical extent.

## Locate and project

Use `locate` to calculate one 3D point from its image coordinates:

```bash
python .agents/skills/spatial-memory/scripts/scene.py locate --input measurement.json
```

With multiple calibrated views, mark the **same stationary physical point** in
each image. The command triangulates its position: it finds where the viewing
lines from each camera through the marked pixels meet most closely.

```json
{"views":[{"camera":"cam_head","pixel":[320,240]},
          {"camera":"cam_left_wrist","pixel":[250,210]}]}
```

With one view, supply a known plane. `height_m` is the point's known perpendicular
distance from that plane, measured along its normal:

```json
{"views":[{"camera":"cam_head","pixel":[320,240]}],
 "plane":"table","height_m":0.03,"object":"item"}
```

A view can include `"observation":"observations/003/observation.json"` to use the
camera calibration recorded with that image, rather than the current camera
pose. Results contain `point_m`; triangulation also reports how closely the rays
meet. Views of a moving point cannot be combined as one measurement.

Supplying `object` writes `point_m` to that object's `position`. Thus, the selected
image point must represent the object's local origin. Omit `object` when measuring
another feature, such as a point on the main surface; only the calculation result
is returned, with no object position changed.

Use `project` for the reverse calculation: where a known 3D target appears in a
camera image.

```bash
python .agents/skills/spatial-memory/scripts/scene.py project --input projection.json
```

```json
{"camera":"cam_head","target":{"object":"item","offset":[0,0,0.02]}}
```

### Target points

The `target` field used by `project`, `pose` and `ik` accepts:

- `[x,y,z]`: a world position.
- `"item"` or `{"object":"item"}`: an object's origin.
- `{"object":"item","offset":[x,y,z]}`: a point relative to that origin.
- `{"object":"item","landmark":"corner"}`: a named point defined in its asset.
- `{"object":"item","bounds":[0.5,0.5,1]}`: a point specified as fractions of
  the object's local bounding box, from 0 (minimum) to 1 (maximum).
- `{"robot":"left"}`: the robot's configured contact point. Optional `link`
  selects another robot link; optional `offset` specifies a point relative to the
  selected link instead of the configured contact point.

An object `offset` is in metres along the object's rotated axes; it can also be
added to a `landmark` or `bounds` target. Landmarks and bounds already include the
object's fitted scale. To read those dimensions and points directly, call
`scene.py geometry --input object.json` with `{"object":"item"}`. Its result
contains `bounds_m`, `dimensions_m` and `landmarks_m` in object-local coordinates.

## Shared assets and instances

An **instance** is one object using an asset at a particular position,
orientation and scale.
`assets` maps asset IDs to `{"file":"assets/id.json"}`; each entry in `objects`
references an `asset` and stores its own world `position`, WXYZ `quaternion`,
local `scale`, and optional `rgba` color. Scale converts asset coordinates to
metres; negative components mirror axes. See [shape assets](modeling.md) for
creating geometry and named landmarks.

```bash
python .agents/skills/spatial-memory/scripts/assets.py create --id asset_id --input asset.json
python .agents/skills/spatial-memory/scripts/assets.py transform --object object_id --translate 0.1 0 0
python .agents/skills/spatial-memory/scripts/assets.py export --object object_id --output object.glb
```

`transform` changes the instance and saves the scene. It accepts world
`--translate dx dy dz`, local `--rotate x y z` in degrees (intrinsic XYZ order),
multiplicative `--scale sx sy sz`, repeatable `--mirror x|y|z`, and
`--color r g b a`. These flags can be combined without changing the shared asset.
`export` writes a mesh at the instance's world placement without changing the scene.

## Fit metric placement and dimensions

Use `fit` to adjust an existing shape's position, orientation and size so its
projected landmarks match measured image pixels. The agent supplies the shape,
identifies matching image points, and selects which parameters to calculate;
the script minimizes the pixel
mismatch and saves the resulting values.

```bash
python .agents/skills/spatial-memory/scripts/scene.py fit --input fit.json
```

Input fields:

- `object`: the scene object to update; its geometry must define landmarks.
- `views`: records of `{camera, pixels: {landmark: [u,v]}}`. Optional `observation`
  selects a saved camera snapshot. Providing both `K` and `T_base_camera` supplies
  calibration directly for that view; optional `distortion` accompanies them.
- `parameters`: any of `position`, `rotation`, `scale`; defaults to all three.
  Use `["position","rotation"]` when established geometry has only moved.
- `scale_mode`: `uniform` (default) preserves proportions with one scale
  multiplier; `axes` allows three independent multipliers; `fixed` keeps scale.
- `geometry_fields`: optional numeric shape parameters to calculate alongside
  placement, such as a part's size or relative position. See
  [internal geometry fitting](modeling.md#fit-internal-geometry) for field paths
  and bounds. Landmarks belonging to a part move with that part's geometry.
- `share_scale_with`: IDs of equal-size objects using the same asset. They receive
  the fitted scale while keeping their own poses. The asset's `shared_fields`
  likewise ties equal internal dimensions together; see the same modeling section.
- `plane`: optional `{name, landmark, height_m}` specifying a landmark's known
  height above a saved plane.
- `metadata`: optional measurement notes saved with the observation source.
- `overlays`: optional output directory for images showing projected geometry and
  measured pixels. Include an `image` path in each view to render it.

The result reports `pixel_rms` (root-mean-square pixel mismatch) and `committed`
(whether the scene was updated). If the selected parameters cannot be determined
from the observations, the scene remains unchanged. Add a view from a different
camera position or hold already-known parameters fixed. Single-view fitting
needs a metric reference, such as known object dimensions or a landmark's known
height above a measured plane. A small pixel mismatch
assumes correct shape, point matches and calibration; it does not verify them.

An object may use `pending_asset` to name a replacement shape while keeping its
current geometry. A committed fit promotes it to `asset`. Direct `asset`
references need no such staging. Once active, the asset replaces inline geometry;
read its fitted dimensions with `geometry`.

## Tool pose and IK

Use `pose` to convert a desired contact point and approach directions into the
position and orientation of the robot link controlled by the motion commands.
Here, the **tool** is the gripper or other device mounted at the arm's end. Its
configured contact offset accounts for the distance from that link to the point
that should reach the target.

```bash
python .agents/skills/spatial-memory/scripts/scene.py pose --input pose.json
```

```json
{"robot":"left","target":{"object":"item","offset":[0,0,0.08]},
 "approach":[0,0,-1],"closing":[1,0,0]}
```

Choose `approach` and `closing` as world-frame directions based on the intended
interaction and geometry. The command combines them with the configured local
gripper axes and contact offset, returning `pose` as `[x,y,z,qw,qx,qy,qz]`. It does
not move the robot; submit this target through the [execution commands](execution.md).

For a joint solution, `scene.py ik --input ik.json` performs **inverse kinematics
(IK)**: calculating joint positions for a desired contact point and orientation.
It accepts `{robot,target,orientation_axes}` plus optional `joint_bounds` and
`seed_weight`; see [robot kinematics](geometry.md#robot-kinematics). It starts from
the measured joints and converts world targets into robot-base coordinates. This
calculation alone does not plan a collision-free motion.

## State updates

Use `update` to save other observed field changes. Its input has the same structure
as `scene.json`: nested fields merge into the existing state, and `null` deletes a
field. For example:

```json
{"objects":{"item":{"position":[0,-0.2,0.78],
                      "quaternion":[1,0,0,0],
                      "source":"observations/003/cam_head.png"}}}
```

Objects can also store simple geometry directly: box `size` is
`[full_x,full_y,full_z]`, sphere `[radius]`, cylinder or capsule `[radius,length]`,
and ellipsoid `[radius_x,radius_y,radius_z]`. Compound geometry uses `parts`, each
with its own local shape, position and orientation. `role: "marker"` denotes a
measurement annotation excluded from solid geometry.

Updating position or orientation records `observed_pose` and clears any previous
`predicted_pose`. If the object is attached to the robot, the update also refreshes
its relative position and orientation, as described below. Existing assets remain
in use when only an object's pose changes.

## Attachments

An **attachment** records that an observed object moves with the gripper. It stores
the object's position and orientation relative to the controlled robot link.
Record it after observing that the robot holds the object:

```bash
python .agents/skills/spatial-memory/scripts/scene.py attach --input attachment.json
python .agents/skills/spatial-memory/scripts/scene.py detach --input release.json
```

Attach input is `{"object":"item","robot":"left"}`; detach input is
`{"object":"item"}`. These commands update scene state, not the physical gripper.
When the robot's measured state is synchronized, an attached object's position is
predicted from the robot motion. `observed_pose` preserves the last observation;
`predicted_pose` records the inferred pose. New measurements update that estimate.
Remove the attachment when the object is no longer held.
