# Shape assets and shared dimensions

Use this document to describe object shapes, save reusable assets, and calculate
their dimensions and placement from images. During initial reconstruction, cover
all observed objects and reproduce their visible structure as faithfully as the
observations allow. Generate independent shapes in parallel as described in the
[workflow](../SKILL.md#workflow). Use the robot's existing model for its geometry.

The agent describes shapes and matches image features; the scripts calculate
numerical parameters. Four terms connect these steps:

- An **asset** is a reusable shape description, stored in `assets/<asset_id>.json`.
- An **instance** is one physical object in `scene.json`'s `objects`. It references
  an asset and has its own position, orientation, scale, and color.
- A **landmark** is a named point on the asset that the agent can identify in an
  image. Matching its image pixels across views supplies measurement data.
- **Fitting** adjusts selected model parameters so projected landmarks align
  with those measured pixels. It calculates placement and dimensions from the
  supplied shape and correspondences.

## Asset format

Write an asset description as JSON with these fields:

- `parts`: the solids that make up the shape. Each part has coordinates relative
  to its parent and optional `position`, `quaternion`, and `scale`. A quaternion
  represents orientation with four numbers in `[w,x,y,z]` order; `[1,0,0,0]`
  means no rotation. Nested `parts` form groups.
- `landmarks`: named points in asset coordinates, or points tied to individual
  parts as described under [Fit internal geometry](#fit-internal-geometry).
- `shared_fields`: optional groups of numeric fields that must stay equal.
  The first field supplies the group's initial value.
- `uncertainty`: optional notes on structure or dimensions unresolved by images.

Asset lengths become metres after applying the instance's scale. Within one
asset, use consistent units. Supported solids are:

| `shape` | Geometry fields |
|---|---|
| `box` | `size`: full x, y, z lengths |
| `sphere` | `size`: radius as a one-element array |
| `ellipsoid` | `size`: x, y, z radii |
| `cylinder` | `size`: radius and length along local z |
| `capsule` | `size`: radius and cylindrical section length; hemispherical ends add to its total length |
| `mesh` | `vertices` and triangular `faces`, or `file` relative to the scene directory |
| `sweep` | `path`: sampled 3D centreline; `radius`: one value or one per point; optional `closed` joins the ends |
| `revolve` | `profile`: ordered `[radius,z]` points rotated about local z |

`sections` sets angular resolution for cylinders, capsules, sweeps, and revolved
shapes. Mesh vertex indices start at zero.

By default, a group assembles its parts. Set `operation: "difference"` on a group
to subtract all later parts from the first. Its children must be closed solids;
subtracted parts retain ordinary dimensions and transforms that can be fitted.

## Create and place an asset

Run these commands from the task workspace, using the Python environment with
the skill's dependencies installed. An existing `scene.json` is required; for a
new workspace, create it once with `printf '%s\n' '{}' > scene.json`.
Here, `asset.json` contains the shape:

```sh
python .agents/skills/spatial-memory/scripts/assets.py create --scene scene.json --id item_shape --input asset.json
python .agents/skills/spatial-memory/scripts/scene.py --scene scene.json update --input instance.json
```

`create` validates the shape, saves `assets/item_shape.json`, and registers its
path under `scene.json`'s `assets`. The second command adds the object instance
to `scene.json` from `instance.json`, with an initial position in metres and
orientation:

```json
{"objects":{"item":{"asset":"item_shape","position":[0,0,0.8],"quaternion":[1,0,0,0],"scale":[1,1,1]}}}
```

Objects with the same shape reference the same asset. Position, orientation,
scale, mirroring, and color can differ between instances; use the
[instance transform commands](scene.md#shared-assets-and-instances) to set them.

When several instances also have equal physical size, list their object IDs in
`share_scale_with` in a fit request for one representative. The calculated scale
is copied to those instances. Fit their positions and rotations separately with
scale fixed. Objects of different sizes retain separate scales.

## Fit internal geometry

Ordinary fitting adjusts the whole object's position, orientation, and scale;
see [placement fitting](scene.md#fit-metric-placement-and-dimensions). To measure
internal dimensions or part positions, select the corresponding asset fields
with `geometry_fields`.

A field path is a sequence of JSON keys and zero-based array indices. For example,
`["parts",0,"size",0]` selects the first size value of the first part. Declare
equal dimensions with `shared_fields` so one measurement applies to repeated
structures while each part keeps its own placement:

```json
{"shared_fields":[[["parts",0,"size",0],["parts",1,"size",0]]]}
```

The agent decides which dimensions are equal from the observed structure.
Record that assumption in the fit request's `metadata`. Unseen members inherit
the measured value through the shared group.

A fit request selects one numeric field per shared group, with allowed minimum
and maximum values in asset units:

```json
{"geometry_fields":[{"path":["parts",0,"size",0],"bounds":[0.2,0.8]}]}
```

Bounds must include the initial value. Keep lengths positive. If fitting instance
scale as well, keep a reference asset dimension fixed: otherwise changing all
asset lengths and inversely changing scale describes the same physical object.
`parameters: []` leaves the instance placement and scale fixed and fits only the
selected internal fields.

Landmarks used to measure changing parts must follow those parts. Entries under
an asset's `landmarks` can use these forms:

```json
{"edge":{"part":[0],"bounds":[1,0.5,0.5]},
 "origin":{"part":[0],"point":[0,0,0]},
 "corner":{"part":[1,0],"vertex":3}}
```

- `part` lists indices through nested `parts` arrays.
- Landmark `bounds` gives fractions of the part's local bounding box, from zero
  at its minimum to one at its maximum. This point changes with part dimensions.
- `point` gives a fixed coordinate within the part.
- `vertex` selects an inline mesh vertex. Part and parent transforms apply to all
  three forms.

Match the same physical feature across views. A curved object's silhouette edge
may show different surface points from different cameras. Unmeasured dimensions
remain estimates unless linked to a measured shared field.

### Invocation and result

Supply landmark pixels measured in the available images. The camera entries in
`scene.json` provide `K` (pixel projection parameters) and `T_base_camera` (camera
position and orientation in the scene's shared coordinate frame). A view can instead name an
`observation` JSON file containing that image's calibration. See
[placement fitting](scene.md#fit-metric-placement-and-dimensions) for camera and
plane inputs, including how to establish metric scale with a single camera.

```sh
python .agents/skills/spatial-memory/scripts/scene.py --scene scene.json fit --input fit.json --output fit_result.json
python .agents/skills/spatial-memory/scripts/scene.py --scene scene.json geometry --input object.json
```

For an instance whose placement and scale are already established, `fit.json`
can measure the selected internal dimension alone:

```json
{"object":"item","parameters":[],"scale_mode":"fixed",
 "geometry_fields":[{"path":["parts",0,"size",0],"bounds":[0.2,0.8]}],
 "views":[{"camera":"cam_a","pixels":{"edge":[210,190]}},
          {"camera":"cam_b","pixels":{"edge":[330,205]}}]}
```

Replace example camera names and pixels with actual observations. Set
`object.json` to `{"object":"item"}` to read the resulting dimensions and landmarks.
These results are in metres along the instance's local axes, before its position
and orientation are applied.

`fit_result.json` reports whether the update was saved (`committed`), the average
projection error (`pixel_rms`, root mean square in pixels), fitted values, and
which field paths changed (`applied_to`). Insufficient measurements to determine
the selected parameters leave the asset and scene unchanged. A committed fit
updates the selected instance and any fitted asset fields; all instances of that
asset then use the revised shape. A small pixel error measures agreement with
supplied landmarks, not completeness of the reconstructed shape.

After reconstruction, movement normally requires only new position and
orientation measurements. For these later fits, use
`"parameters": ["position", "rotation"]` and omit `geometry_fields` to retain
established geometry and scale.
