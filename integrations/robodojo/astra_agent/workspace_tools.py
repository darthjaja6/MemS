"""ARX observation and waypoint conversion; task reasoning stays with the agent."""
import copy
import json
from pathlib import Path
import sys
import yaml
import numpy as np

HERE = Path(__file__).resolve().parent
SKILL = HERE / '.agents/skills/spatial-memory/scripts'
if not SKILL.exists():
    SKILL = HERE.parents[2] / 'skills/spatial-memory/scripts'
sys.path.insert(0, str(SKILL))
import scene

COMPLETION_STEPS = 48


def check_budget(state, steps, finishing=False):
    """Keep benchmark completion inside the native episode budget."""
    budget = state.get('control_budget')
    if budget and not state.get('terminal'):
        remaining = budget['step_limit'] - budget['steps_used']
        available = remaining if finishing else max(0, remaining - COMPLETION_STEPS)
        if steps > available:
            raise ValueError(f'Sequence needs {steps} steps; {available} available. '
                             f'{COMPLETION_STEPS} steps are reserved for the finish decision and return home.')


def initialize(workspace, observation):
    config = yaml.safe_load((workspace / 'robot/robot_config.yml').read_text())
    robots = {}
    for side in ('left', 'right'):
        robots[side] = dict(urdf=str(workspace / 'robot/X5A.urdf'), tip=config['ee_link'],
            joint_names=config['arm_joints_name'],
            gripper_joints=config['gripper_joints_name'], gripper_scale=config['gripper_scale'],
            tool=dict(point_in_tip=[config['gripper_bias'], 0, 0],
                      local_approach=[1, 0, 0], local_closing=[0, 1, 0]))
    state = dict(frame='world', units='metres', control_period_s=.04, robots=robots, assets={}, objects={}, surfaces={},
                 initial_state=observation['state'])
    return scene.sync(state, observation)


def publish(workspace, state):
    scene.save(workspace / 'scene.json', state)


def synchronize(workspace, observation, last_command=None):
    path = workspace / 'scene.json'
    state = json.loads(path.read_text()) if path.exists() else initialize(workspace, observation)
    scene.sync(state, observation)
    if last_command is not None:
        state['tool_commands'] = {s: float(last_command[s + '_ee_joint_state'][0])
                                  for s in ('left', 'right')}
    publish(workspace, state)


def prepare(workspace, request):
    state = json.loads((workspace / 'scene.json').read_text())
    scene.observe(state, request.get('scene_update', {}))
    for item in request.get('attachments', []):
        if item.get('robot') is None:
            state['objects'][item['object']].pop('attachment', None)
        else:
            scene.attach(state, **item)
    # Keep predicted waypoint targets separate from measured scene state.
    current = {s: dict(pose=state['robots'][s]['pose'],
                      gripper=state.get('tool_commands', {}).get(s, state['robots'][s]['gripper']))
               for s in ('left', 'right')}
    waypoints = []
    actions = list(request.get('actions', []))
    # Completion in this benchmark includes restoring the initial arm poses.
    completing = bool(request.get('finish')) and not state.get('terminal')
    if completing and not (actions and actions[-1].get('home')):
        actions.append(dict(home=True, steps=COMPLETION_STEPS, checkpoint='Completion'))
    for action in actions:
        constraints = {}
        for side in current:
            if action.get('home'):
                initial = state['initial_state']
                current[side] = dict(pose=initial[side + '_ee_pose'],
                                     gripper=initial.get(side + '_ee_joint_state', [1])[0])
            command = action.get(side, {})
            if 'target' in command and 'pose' in command:
                raise ValueError('Supply a contact target or a full pose, not both')
            if any(axis in command for axis in ('approach', 'closing')) and 'target' not in command:
                raise ValueError('Tool-axis constraints require a contact target')
            previous_pose = current[side]['pose']
            constraints[side + '_move'] = bool(action.get('home') or 'target' in command or 'pose' in command)
            constraints[side + '_axes'] = None
            if 'target' in command:
                rotation = scene.pose_matrix(current[side]['pose'])[:3, :3]
                tool = state['robots'][side]['tool']
                # A single supplied axis is enforced by IK. Keep a complete reference
                # orientation without inventing a second required direction.
                approach = rotation @ tool['local_approach']
                closing = rotation @ tool['local_closing']
                if 'approach' in command and 'closing' in command:
                    approach, closing = command['approach'], command['closing']
                constraints[side + '_axes'] = [[tool['local_' + axis], command[axis]]
                                               for axis in ('approach', 'closing') if axis in command]
                current[side]['pose'] = scene.pose(state, side, command['target'],
                                                  approach, closing, root=workspace)
                if not constraints[side + '_axes'] and np.allclose(current[side]['pose'], previous_pose,
                                                                   atol=1e-10, rtol=0):
                    constraints[side + '_move'] = False
            elif 'pose' in command:
                current[side]['pose'] = command['pose']
            if 'gripper' in command:
                current[side]['gripper'] = command['gripper']
        waypoint = {f'{s}_{key}': copy.deepcopy(value) for s, values in current.items()
                    for key, value in values.items()}
        waypoints.append(dict(**waypoint, **constraints, steps=action['steps'], checkpoint=action.get('checkpoint', '')))
    if 'waypoints' in request:
        if waypoints:
            raise ValueError('Supply actions or waypoints, not both')
        waypoints = request['waypoints']
    if not waypoints and not any(request.get(k) for k in ('finish', 'scene_update', 'attachments')):
        raise ValueError('Supply a scene update, actions, attachments or a finish decision')
    # Refuse accidental truncation of a declared sequence.
    if sum(w['steps'] for w in waypoints) > 96:
        raise ValueError('A sequence may contain at most 96 control steps')
    if any(w.get('checkpoint') for w in waypoints[:-1]):
        raise ValueError('Place the review checkpoint at the end of the submitted sequence')
    state['plan'] = dict(actions=actions, waypoints=waypoints,
                         finish=request.get('finish'))
    publish(workspace, state)
    return dict(waypoints=waypoints, contacts=request.get('contacts', []), finishing=completing)
