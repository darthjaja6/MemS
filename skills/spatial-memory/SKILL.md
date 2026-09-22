---
name: spatial-memory
description: >-
  Maintains the spatial information of the environment, and a
  set of scripts for the agent to call to calculate target position(s) of the robot in order to complete tasks
---

# MemS

Spatial memory describes the geometry and spatial relationships of the
robot and its environment. It is stored in `scene.json`, which includes:

1. The robot model, joint states, and camera calibration.
2. Object positions, orientations, shapes, and dimensions.

During task execution, use and update `scene.json` as an estimated digital
twin of the physical environment. Use this information and the provided
scripts and follow the workflow below to complete the task. Do not write your own code or modify
the scripts.

In `references/example_scene.json`, there's a example of the scene.json file created with mock data. Comments explain the fields. Your actual scene should follow the pattern but do not use any data from the file.

## Workflow

1. **Observe and reconstruct the scene.** 
Observe the environment, reconstruct the 3D scene, generate the assets and save the data into `scene.json` file. This is a one-time effort only executed at the beginning of this workflow. In later steps, only the position, orientation, etc. of the objects will be updated in `scene.json`.
a. Understand the robot model and get its parameters. Use [`kinematics.py inspect`](references/geometry.md#robot-kinematics) to get the robot's joints, links, and limits from its URDF model.
b. Locate the main surface relative to the robot to establish a reference for object positions and movement heights above the surface. Follow the [plane measurement instructions](references/scene.md), choosing the method appropriate to the available cameras and calibration data, and save the resulting plane in `scene.json`.

c. Reconstruct all observed objects with high-fidelity [shape assets](references/modeling.md):
   - Identify unique shapes. Reuse assets for objects related by rotation, scaling, mirroring, or color changes; apply these instance changes with [`assets.py transform`](references/scene.md#shared-assets-and-instances). For repeated substructures of one object/shape sharing equal dimensions, model just once and replicate them.
   - Start parallel subagents to generate the unique shapes. Use [`assets.py create`](references/modeling.md#create-and-place-an-asset) to save them to `assets/<asset_id>.json` in the task workspace and register their paths under `assets` in `scene.json`.
   - Create each instance under `objects` with an `asset` reference. Use the [geometry fitting scripts](references/scene.md#fit-metric-placement-and-dimensions) and calibrated observations to calculate and save its position, orientation, and dimensions, including shared substructure dimensions.

2. **Plan and execute an action sequence.**

   Review the current observation against the task goal.

   - If the task is complete, end the workflow.
   - Otherwise, plan the next action sequence: specify key target positions
     and orientations for the gripper or other device mounted at the end of
     the arm, along with its operations. For a gripper, choose approach and
     jaw-closing directions based on its geometry and the object's shape so the object won't flip.
     Use [`scene.py pose`](references/scene.md#tool-pose-and-ik) when you need to
     convert a target point and these directions into a robot end-effector pose;
     it accounts for the gripper's offset from the controlled robot link.
   - Use the [motion planning and execution scripts](references/execution.md) to
     calculate a path from the robot's current measured joint state through
     these targets and execute it using the documented action format and command
     for the robot. Then observe the result. Recalculate changed object poses with
     [`scene.py locate` or `scene.py fit`](references/scene.md), specifying which
     object to update. Use [`scene.py update`](references/scene.md#state-updates)
     for other observed field changes, and [`attach` or `detach`](references/scene.md#attachments)
     to record whether an object moves with the gripper. Repeat this step.

To visualize the scene or replay recorded movements, see [visualize.md](references/visualize.md).
