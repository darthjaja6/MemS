# RoboDojo integration

Run one locally authenticated Codex agent per native RoboDojo episode. Each agent receives the complete spatial-memory skill, robot geometry, camera images and calibration, measured joint state, and a file-based action bridge. It updates `scene.json`, submits action sequences, and decides when it has finished. Native benchmark scoring remains authoritative.

## Requirements

- A working [RoboDojo](https://github.com/RoboDojo-Benchmark/RoboDojo) checkout, including XPolicyLab and downloaded ARX X5 assets.
- Its working Isaac Sim / Isaac Lab Python environment. This integration was exercised with Python 3.11 and Isaac Sim 5.1 on Linux.
- An authenticated Codex CLI and access to the model selected in `deploy.yml` (`gpt-6-astra` by default). This uses the local CLI login, not a separate API key. Inference still uses the model service.

Set up the simulator, assets, and official example through RoboDojo's own instructions first. The installer below adds policy dependencies and links the adapter into that existing checkout; it does not install Isaac Sim or download benchmark assets.

## Install and run

From the `spatial_memory` directory, activate your working RoboDojo virtualenv or conda environment, then:

```bash
export ROBODOJO_ROOT=/absolute/path/to/RoboDojo
bash integrations/robodojo/install.sh
python integrations/robodojo/agent_pilot.py \
  --output runs/stack_blocks --tasks stack_blocks
python integrations/robodojo/agent_report.py runs/stack_blocks
```

The installer installs pinned motion dependencies, creates `XPolicyLab/policy/astra_agent`, and applies the transport patches to load policy settings in the evaluation client and forward its configured request timeout. It stops if the patch no longer matches the upstream checkout or a different adapter already occupies the target path.

Use `--tasks task_a task_b` to select tasks and `--seed N` to choose the native seed. The default runs six tasks, one episode each. Repeating a command resumes its run and skips completed tasks; use a new output directory after changing the skill, policy, or seed. Outputs include native scores, video paths from the benchmark, action logs, task-agent token usage, and elapsed time. Action chunks are not model-call counts.

## Task interface

The agent receives the [command interface](interface.md) as `references/robot-interface.md` within its task-workspace skill. It writes JSON and calls `python robot.py decision.json`. One request can include scene updates, attachment changes, actions, checkpoints and a finish decision. Prepared commands compute geometry and submit joint trajectories. The agent supplies tool targets and task-relevant endpoint directions; it does not implement a planner.

The adapter initializes robot geometry and base transforms, and synchronizes measured state in `scene.json` after each action chunk. Camera matrices come from the active pinhole camera and the tiled RGB renderer. The task agent describes relevant geometry and uses calibrated measurements to locate it. Reusable assets and internal dimension fitting are available when additional detail helps the action. The live viewer reads the same scene and its recorded trajectories. Object poses and evaluator rewards are not supplied to the agent. This supplemental camera metadata should be checked against the input contract of any hosted evaluation.

An episode uses a separate Git workspace and Codex home. The CLI runs with filesystem access; task-input restrictions in the command reference are instructions, not OS isolation. Existing file-based Codex authentication is copied into the episode home and removed on normal completion. Keep `runs/` private.

## Planning

The shared [motion planner](../../../skills/spatial-memory/scripts/motion_planning/) prepares robot models for Drake IK and MPlib/OMPL routes, then uses PCHIP/TOPPRA for timing. Every sequence starts from measured joints. This adapter supplies ARX X5 gripper encoding, 25 Hz control, and native sequence limits.

Position-only targets treat the previous orientation as a preference. Supplied axes and full poses constrain the endpoint. These constraints do not enforce a constant orientation throughout the path. The adapter checks self/inter-arm, scene mesh and held-object collisions. Intentional contacts are explicit pairs. Each gripper phase plans with its requested opening and checks the opening transition at 1 mm intervals along the sampled arm path. The check samples a linear transition of both grippers together and allows that transition anywhere along the phase. It does not model actuator dynamics or independently delayed fingers, and can reject paths that would be safe with precise closure timing. Subsequent phases use the preceding requested opening until the next measured observation.

The execution interface reserves 48 native control steps for completion. A finish request returns both arms to their initial poses; measured joints are then held while native scoring completes. Terminal observations let it record the final scene. Post-episode native diagnostics stay outside the task workspace.

## Local check

```bash
PYTHONPATH="$ROBODOJO_ROOT${PYTHONPATH:+:$PYTHONPATH}" \
  python -m unittest integrations.robodojo.astra_agent.test_bridge integrations.robodojo.astra_agent.test_motion
```
