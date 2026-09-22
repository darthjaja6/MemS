"""Supply observations and execute the workspace agent's action chunks."""
import json
import os
from pathlib import Path
import time
import numpy as np
from scipy.spatial.transform import Rotation


def rendered_intrinsics(camera, image_shape):
    """RoboDojo's tiled pinhole renderer uses horizontal aperture and square pixels.

    Read the active camera rather than the template. The SDK's independent
    vertical-aperture calculation can disagree with this RGB render path.
    """
    if camera.get_lens_distortion_model() != 'pinhole':
        raise ValueError('Tiled camera calibration requires a pinhole camera')
    width, height = camera.get_resolution()
    if tuple(image_shape[:2]) != (height, width):
        raise ValueError('Camera resolution and delivered image dimensions differ')
    focal = width * camera.get_focal_length() / camera.get_horizontal_aperture()
    if not np.isfinite(focal) or focal <= 0:
        raise ValueError('Invalid active camera focal length or aperture')
    return np.array([[focal, 0, width / 2], [0, focal, height / 2], [0, 0, 1]])


def observation(env, steps, terminal=False, obs=None):
    if obs is None:obs=env.get_obs()
    obs['control_budget']=dict(steps_used=steps,step_limit=env.step_lim)
    obs['terminal']=terminal
    calibration={}
    for index,camera in enumerate(env.camera_manager.cameras[0]):
        name=env.camera_manager.camera_names[0][index]
        position,quat=camera.get_world_pose(camera_axes='ros')
        T=np.eye(4)
        T[:3,:3]=Rotation.from_quat(np.asarray(quat),scalar_first=True).as_matrix()
        T[:3,3]=np.asarray(position)
        calibration[name]=dict(K=rendered_intrinsics(camera, obs['vision'][name]['color'].shape).tolist(),
                               T_base_camera=T.tolist(), resolution=list(camera.get_resolution()),
                               source='active camera / RoboDojo tiled pinhole RGB')
    obs['camera_calibration']=calibration
    return obs


def eval_one_episode(TASK_ENV, model_client):
    env=TASK_ENV
    model_client.call(func_name='reset')
    steps=0
    obs=observation(env,steps)
    while not env.is_episode_end():
        model_client.call(func_name='update_obs',obs=obs)
        started=time.monotonic()
        try:
            actions=model_client.call(func_name='get_action')
        except Exception as error:
            raise SystemExit(f'Policy request failed: {error}') from error
        execution=time.monotonic()
        start_step=steps
        for action in actions:
            env.take_action(action)
            steps+=1
            obs=env.get_obs()
            if env.is_episode_end():break
        obs=observation(env,steps,terminal=env.is_episode_end(),obs=obs)
        with (Path(os.environ['SPATIAL_RUN_ROOT'])/'execution.jsonl').open('a') as stream:
            stream.write(json.dumps(dict(step_start=start_step,step_end=steps,
                 policy_call_s=execution-started,execution_s=time.monotonic()-execution))+'\n')
    model_client.call(func_name='finish',obs=obs)
    # Post-episode diagnostics stay outside the policy workspace and observations.
    rm=env.reward_manager
    with (Path(os.environ['SPATIAL_RUN_ROOT']).parent/'native_diagnostics.json').open('w') as stream:
        json.dump(dict(remaining_checks=rm.check_list,queries=rm.query_list,
                       score_completed_count=rm.score_completed_count,
                       score_achieved=rm.score_achieved),stream,indent=2,
                  default=lambda value:value.tolist() if hasattr(value,'tolist') else str(value))
