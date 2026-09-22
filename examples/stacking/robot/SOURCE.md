# ARX X5 display model

The replay uses the recorded joint values and base transforms. Its URDF joint
frames match those used in the recording.

- Body meshes (`base_link.STL`, `link1.STL` through `link6.STL`) come from
  [real-stanford/arx5-sdk](https://github.com/real-stanford/arx5-sdk/tree/a8890c9bae94464abd1cb7c5e4da7c4a62104a3a/models/meshes/X5),
  commit `a8890c9bae94464abd1cb7c5e4da7c4a62104a3a`, under MIT; see `SDK-LICENSE`.
- `X5A.urdf` and the two finger meshes come from
  [RoboTwin2.0's ARX-X5 assets](https://huggingface.co/datasets/TianxingChen/RoboTwin2.0/tree/d0c7738c868de0596a52f13d5d62775965558162/embodiments/ARX-X5),
  revision `d0c7738c868de0596a52f13d5d62775965558162`, declared MIT by that dataset.
  The [RoboTwin project](https://github.com/RoboTwin-Platform/RoboTwin) license
  is retained in `ROBOTWIN-LICENSE`.

Large body meshes are simplified with quadric edge collapse for interactive
display (target 6,000 triangles per mesh). Finger meshes are unmodified.
The URDF is reformatted, camera visuals are omitted, and the camera joint origin
is set to the value in the recorded robot model. Body meshes are the SDK's visual
model, rather than a claim of exact simulator appearance. No benchmark object
assets are included.
