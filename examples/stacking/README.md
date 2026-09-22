# Recorded stacking example

These are estimated scene states from a real agent run in RoboDojo’s simulated
`stack_blocks` task (seed 2, September 21, 2026). This is a simulation recording,
not footage of a physical robot.

`scene.json` holds the final memory. `replay.json` retains nine scene snapshots
and robot joints from recorded commands and measured checkpoints.
The viewer calculates both arms and their tool positions from those joints.
Playback starts at the first populated scene; its clock shows simulator motion
time, excluding time spent reasoning. Object poses change only at recorded scene
updates. No motion between object snapshots is invented.

All four objects reuse the agent-created box asset. Their dimensions, poses,
and colors are preserved from the run. The [ARX X5 display model](robot/SOURCE.md) includes attributed robot geometry.
Camera images and benchmark object assets are omitted. The two arms and their
color-coded tool paths can be inspected throughout the replay.

This sample illustrates changing spatial memory; the box approximation is not
a claim of high-fidelity reconstruction or simulator ground truth.
