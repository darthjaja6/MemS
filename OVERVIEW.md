# How MemS works

MemS stores an agent's working model of a place as explicit geometry
and state. An object can keep its shape while its location changes; several
objects can share one shape. New observations revise the model, and the agent
can inspect or query it between observations.

## The memory model

| Layer | Representation | Purpose |
| --- | --- | --- |
| Shared geometry | `assets/*.json` | Reusable shapes, named landmarks, and linked dimensions |
| Scene state | `scene.json` | Object instances, surfaces, cameras, and optional robot state |
| Observations and history | Deployment recordings | Evidence for updates and a record of what happened |
| Inspection | Browser viewer and geometry exports | Read the scene and inspect recorded sessions |

An **asset** describes a shape in local coordinates. An **instance** references
that asset and supplies position, orientation, scale, and optionally color.
Changing an asset changes the shared shape; moving one instance changes only
that object's placement. Named landmarks connect geometry to recognizable points
in images.

World positions use metres in a common scene frame. Orientations use WXYZ
quaternions. Camera calibration relates image pixels to that frame. A surface
plane provides a measurement reference; an object can separately describe its
finite physical extent.

## From observations to an updated scene

The agent identifies objects, proposes shapes, and marks corresponding image
points. The tools calculate the numerical consequences:

1. **Locate points.** Triangulate matching points across calibrated views, or
   intersect one camera ray with a known plane.
2. **Fit geometry.** Adjust selected position, orientation, scale, or internal
   shape parameters to align projected landmarks with measured pixels.
3. **Save and reuse.** Commit the fitted state, query dimensions or target points,
   and reuse the geometry for subsequent observations.

The agent chooses which parameters are supported by the evidence. A single view
needs a metric reference to recover scale. If the selected parameters cannot be
determined from the measurements, fitting leaves the scene unchanged. A low
projection error still depends on correct shape assumptions, correspondences,
and calibration.

See [scene commands](skills/spatial-memory/references/scene.md) and
[shape assets](skills/spatial-memory/references/modeling.md) for the data contracts.

## Measured state and predicted state

Scene updates record an object's observed pose. An attachment can also describe
its pose relative to a robot link: as measured robot joints change, the object's
new pose is predicted from that relationship. `observed_pose` preserves the last
observation, while `predicted_pose` records the inference. A new object observation
updates the pose and clears the old prediction.

That distinction lets the agent revisit assumptions such as whether a gripper
still holds an object. Recorded execution history remains separate from the
current working scene. The read-only viewer exposes both scene state and recorded
sessions without commanding a robot.

## Optional action loop

Robot deployments connect the memory to observations and a controller. The agent
chooses targets, approach directions, and checkpoints; geometry and motion tools
convert those decisions into joint trajectories. At a checkpoint, fresh feedback
can change the scene and the next decision.

Planning uses measured starting joints and supplied collision geometry. Its
numerical checks describe the reconstructed model; they do not establish physical
execution accuracy. The included RoboDojo adapter is one deployment of this loop.
The geometry tools and viewer can be used independently.

For setup and entry points, see the [README](README.md). The complete
[agent skill](skills/spatial-memory/SKILL.md) defines the operational workflow.
