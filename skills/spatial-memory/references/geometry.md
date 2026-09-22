# Geometry commands

Use these scripts to inspect a robot model or calculate geometry from explicit
numeric inputs. They print results as JSON; they do not update `scene.json` or
move the robot. When cameras, planes, or gripper dimensions are already in
`scene.json`, use the [scene commands](scene.md) to read those values and calculate
the result in one call. You do not need to repeat the calculation here.

## Coordinates and units

A **coordinate frame** is an origin and three axes used to express positions and
directions. Keep related inputs in the same frame. Lengths use metres; angles use
radians unless a field ends in `_deg`, which means degrees. Pixels use `[u, v]`
(column, row).

- `T_parent_child` is a 4×4 matrix that converts points from the child frame into
  the parent frame. In camera commands, `T_base_camera` places the camera in the
  chosen base frame; 3D inputs and results use that base frame.
- The camera's local axes are x right, y down, z forward. `K` is its 3×3
  **intrinsic matrix**, containing focal lengths and the optical centre in pixels.
  `T_base_camera` supplies its **extrinsic calibration**: position and orientation
  relative to the base. Use calibration for the actual image resolution and
  orientation. Optional `distortion` contains OpenCV lens-distortion coefficients,
  such as `[k1,k2,p1,p2,k3]`. For already-undistorted images, supply their adjusted
  `K` and omit `distortion`.
- A **pose** combines position and orientation. The pose array is
  `[x, y, z, qw, qx, qy, qz]`. The last four numbers form a **quaternion**, a
  rotation representation in WXYZ order.
- A plane satisfies `normal · point + offset_m = 0`. `normal` points perpendicular
  to the surface. For a unit normal, `offset_m` specifies the plane's signed
  offset from the origin. Optional `height_m` shifts it along its normal.

## Geometry calculations

Pass a JSON file to `geometry.py`, or omit `--input` to read JSON from standard
input. Paths below are relative to the skill directory; from a task workspace,
prepend `.agents/skills/spatial-memory/`.

```bash
python scripts/geometry.py plane --input points.json
python scripts/geometry.py triangulate --input views.json
python scripts/geometry.py tool-pose --input contact.json
```

A camera **ray** starts at the camera and passes through an image pixel. Plane
intersection locates a point on a known surface; **triangulation** locates the same
stationary feature using rays from cameras at different positions.

| Command and purpose | JSON fields | Result |
| --- | --- | --- |
| `plane`: fit a plane to 3D points | `points`, optional known `normal` | `normal`, `offset_m`, `fit_rms_m` |
| `ray`: convert a pixel to a 3D ray | `pixel`, `K`, `T_base_camera`, optional `distortion` | `origin`, unit `direction` |
| `ray-plane`: intersect a ray and plane | `origin`, `direction`, `normal`, `offset_m`, optional `height_m` | `point_m`, `distance_m` along the ray |
| `pixel-plane`: locate a pixel on a plane | `pixel`, `K`, `T_base_camera`, `normal`, `offset_m`, optional `height_m`, `distortion` | `point_m`, `distance_m` from camera |
| `triangulate`: locate a feature across views | `views`: list of `{pixel, K, T_base_camera}`, optional `distortion` in each view | `point_m`, `ray_rms_m`, `max_parallax_deg` |
| `project`: find the pixel for a 3D point | `point`, `K`, `T_base_camera`, optional `distortion` | `[u, v]` with the supplied lens distortion, or undistorted if omitted |
| `transform`: convert coordinates | `T`, `points` | transformed point or points |
| `distance`: measure between two points | `a`, `b` | `distance_m` |
| `tool-pose`: place and orient the gripper or other end-mounted device | `target`, `approach`, `closing`, optional `point_in_tip`, `local_approach`, `local_closing` | `pose` |

For `plane`, supply at least three non-collinear points unless `normal` is already
known. For example, `points.json` can contain:

```json
{"points":[[0,0,0.75],[0.3,0,0.75],[0,0.3,0.75]]}
```

`fit_rms_m` and `ray_rms_m` are root-mean-square distances from the supplied points
to the fitted plane, or from the reconstructed point to its viewing rays.
`max_parallax_deg` is the largest angle between those rays. These describe the
fit to the supplied observations; they do not verify camera calibration.

### Calculate a gripper pose

Choose the desired `target` contact point, `approach` direction and jaw `closing`
direction in the base frame. A **link** is a rigid robot part; the **tip** is the
link whose pose is controlled.
`point_in_tip` is the contact point relative to that link; `local_approach` and
`local_closing` describe the gripper's directions in that link's frame.

For example, `contact.json` can contain:

```json
{"target":[0,-0.2,0.8],"approach":[0,0,-1],"closing":[1,0,0],
 "point_in_tip":[0.145,0,0],"local_approach":[1,0,0],"local_closing":[0,1,0]}
```

Each approach/closing pair must be perpendicular. The script aligns the directions
and accounts for the contact-point offset to return the tip pose. The optional
local values default to `[0,0,0]`, `[1,0,0]`, and `[0,1,0]`, respectively; supply
the actual gripper geometry. Use `scene.py pose` when it is already configured.

## Robot kinematics

**Kinematics** calculates robot link positions from joint values, or joint values
needed to reach a position. `kinematics.py` reads a **URDF**, the robot model file
that defines rigid links, their connecting joints, and joint limits.

| Command | Use and result |
| --- | --- |
| `inspect` | Read the model's root link, joints, limits, and visual geometry. |
| `fk` (forward kinematics) | Calculate link transforms from joint values. |
| `ik` (inverse kinematics) | Find joint values that place a selected link or a point on it at a target. |
| `bounds` | Calculate each selected link's minimum and maximum coordinates from its visual geometry. |
| `clearance` | Sample robot geometry along joint movements and report each link's minimum signed distance to a plane. |

```bash
python scripts/kinematics.py inspect --urdf robot.urdf
python scripts/kinematics.py fk --urdf robot.urdf --tip tool_link --q '{"joint1":0.2}'
python scripts/kinematics.py ik --urdf robot.urdf --tip tool_link \
  --q '{"joint1":0.2}' --target 0.2 0 0.1 --point-in-tip 0.1 0 0 \
  --orientation-axes '[{"name":"approach","local":[1,0,0],"world":[0,0,-1],"tolerance_deg":10}]'
```

Replace the example model path, link name, joint values and target. `--q` maps
joint names to measured values: radians for rotating joints, metres for sliding
joints. For IK, these values also provide the **seed**, the starting configuration
for finding a solution. `--point-in-tip` selects the point on the link to position;
omitting it targets the link origin.

Standalone targets and returned transforms use the robot's root/base frame.
`scene.py ik` instead accepts the scene's world frame, the shared reference used
for scene objects. In `--orientation-axes`, `local` is a direction on the selected
link and `world` is its desired direction in the standalone solver's base frame.
Each `tolerance_deg` gives the allowed angular error.

Optional `--joint-bounds '{"joint1":[-1,1]}'` narrows the model's limits.
`--seed-weight` controls the preference for joint values near the seed.
`reachable_in_model` is true when position error is below 3 mm and every supplied
orientation tolerance passes. IK returns a joint solution, not a motion path;
use the [execution scripts](execution.md) to plan and execute movement.

### Measure bounds or plane clearance

```bash
python scripts/kinematics.py bounds --urdf robot.urdf --q '{}' --links tool_link
python scripts/kinematics.py clearance --urdf robot.urdf --input clearance.json
```

`clearance.json` specifies joint states to connect by linear interpolation and
the plane to measure against:

```json
{"waypoints":[{"joint1":0},{"joint1":0.2}],"normal":[0,0,1],
 "offset_m":0,"links":["tool_link"],"samples_per_segment":16}
```

`links` selects robot links; omit it to include all visual geometry.
`samples_per_segment` defaults to 16. Optional `extra_geometry` maps link names to
lists of local vertices for attached objects. Negative clearance means sampled
geometry crosses the plane toward the side opposite its normal. This calculation
samples visual geometry against one plane; it does not check all scene collisions.
