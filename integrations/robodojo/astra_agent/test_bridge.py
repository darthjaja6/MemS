"""Check the asynchronous agent/simulator handshake without another model call."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import unittest
from unittest.mock import patch
import numpy as np
from .model import Model, agent_environment
from .workspace_tools import prepare, scene, synchronize, check_budget

class BridgeTests(unittest.TestCase):
    def test_native_budget_reserves_completion(self):
        state = dict(control_budget=dict(steps_used=458, step_limit=550))
        check_budget(state, 44)
        with self.assertRaises(ValueError):
            check_budget(state, 45)
        state['control_budget']['steps_used'] = 502
        with self.assertRaises(ValueError):
            check_budget(state, 1)
        check_budget(state, 48, finishing=True)

    def test_rendered_camera_intrinsics_follow_active_settings(self):
        from types import SimpleNamespace
        from .deploy import rendered_intrinsics
        camera=SimpleNamespace(get_lens_distortion_model=lambda:'pinhole',
            get_resolution=lambda:(640,480),get_focal_length=lambda:10,
            get_horizontal_aperture=lambda:22.212)
        # Independent rendered-landmark observations from the native tiled path.
        K=rendered_intrinsics(camera,(480,640,3))
        for point, pixel in [([.5,-.5,2],[392.0618,167.9944]),
                             ([-.5,.5,2],[247.9494,312.0393])]:
            projected=K@np.array(point)
            self.assertLess(np.linalg.norm(projected[:2]/projected[2]-pixel),.1)
        camera.get_resolution=lambda:(1280,960)
        doubled=rendered_intrinsics(camera,(960,1280,3))
        np.testing.assert_allclose(doubled[:2],2*K[:2])
        with self.assertRaises(ValueError):rendered_intrinsics(camera,(480,640,3))

    def test_launch_environment_has_no_inherited_session_or_run_context(self):
        with patch.dict(os.environ, CODEX_THREAD_ID='outer', CODEX_SESSION_ID='outer',
                        ROBODOJO_ROOT='/private/benchmark', SPATIAL_RUN_ROOT='/private/runs'):
            env=agent_environment(Path('/task/.codex-home'))
        self.assertEqual(env['CODEX_HOME'],'/task/.codex-home')
        for key in ('CODEX_THREAD_ID','CODEX_SESSION_ID','ROBODOJO_ROOT','SPATIAL_RUN_ROOT'):
            self.assertNotIn(key,env)

    def test_launch_separates_image_options_from_stdin_prompt(self):
        with tempfile.TemporaryDirectory() as directory, patch.dict(os.environ, SPATIAL_RUN_ROOT=directory):
            model=Model(dict(env_cfg_type='arx_x5',action_type='joint',task_name='launch_test'))
            model.reset()
            model.observation={'cameras':{'head':{'image':'head.png'}}}
            with patch('integrations.robodojo.astra_agent.model.subprocess.Popen') as launch:
                model._launch()
                command=launch.call_args.args[0]
                self.assertEqual(command[-2:],['--','-'])
                self.assertEqual(command[command.index('--image')+1],str(model.workspace/'head.png'))
                launch.return_value.stdin.write.assert_called_once()
                self.assertIn("references/robot-interface.md", launch.return_value.stdin.write.call_args.args[0])
                self.assertNotIn('CODEX_THREAD_ID',launch.call_args.kwargs['env'])
            (model.codex_home/'auth.json').unlink()

    def test_complete_skill_and_action_feedback(self):
        with tempfile.TemporaryDirectory() as directory, patch.dict(os.environ, SPATIAL_RUN_ROOT=directory):
            model=Model(dict(env_cfg_type='arx_x5',action_type='joint',task_name='bridge_test'))
            model.reset()
            skill=model.workspace/'.agents/skills/spatial-memory'
            self.assertTrue((skill/'references/geometry.md').is_file())
            self.assertTrue((skill/'references/robot-interface.md').is_file())
            self.assertFalse((model.workspace/'AGENTS.md').exists())
            self.assertTrue((skill/'scripts/geometry.py').is_file())
            self.assertFalse((skill/'references/memory.md').exists())
            state={f'{side}_ee_pose':np.array([0,0,1,1,0,0,0]) for side in ('left','right')}
            state.update({f'{side}_arm_joint_state':np.zeros(6) for side in ('left','right')})
            obs=dict(state=state,vision={},instruction='Test',control_budget=dict(steps_used=0,step_limit=50))
            model.update_obs(obs)
            saved=json.loads((model.workspace/'scene.json').read_text())
            self.assertEqual(set(saved['robots']), {'left','right'})
            plan={'waypoints':[dict(left_pose=[.1,0,1,1,0,0,0],right_pose=[0,0,1,1,0,0,0],
                                  left_gripper=1,right_gripper=1,steps=2,checkpoint='Observe')]}
            (model.workspace/'plan.json').write_text(json.dumps(plan))
            process=subprocess.Popen([sys.executable,'robot.py','plan.json'],cwd=model.workspace,stdout=subprocess.PIPE,text=True)
            model.process=process
            # Bridge protocol test; the recorded-state regressions exercise actual kinematics.
            joint_action={f'{s}_arm_joint_state':np.zeros(6) for s in ('left','right')}
            joint_action.update({f'{s}_ee_joint_state':np.array([1.]) for s in ('left','right')})
            with patch('integrations.robodojo.astra_agent.model.MotionPlanner') as planner:
                planner.return_value.compile.return_value=[joint_action, joint_action]
                actions=model.get_action()
                planner.return_value.compile.assert_called_once()
            self.assertEqual(len(actions),2)
            obs['terminal']=True
            obs['control_budget']['steps_used']=2
            model.finish(obs)
            reply=json.loads(process.stdout.read())
            process.stdout.close()
            self.assertTrue(reply['terminal'])
            self.assertEqual(reply['control_budget']['steps_used'],2)
            self.assertFalse((model.codex_home/'auth.json').exists())

    def test_scene_targets_and_attachment_follow_measured_tool(self):
        from scipy.spatial.transform import Rotation
        with tempfile.TemporaryDirectory() as directory, patch.dict(os.environ, SPATIAL_RUN_ROOT=directory):
            model=Model(dict(env_cfg_type='arx_x5',action_type='joint',task_name='geometry_test'))
            model.reset()
            workspace=model.workspace
            model.reset()
            self.assertEqual(model.workspace, workspace)
            state={f'{side}_ee_pose':[0,0,1,1,0,0,0] for side in ('left','right')}
            state.update({f'{side}_arm_joint_state':[0]*6 for side in ('left','right')})
            obs=dict(state=state,vision={},instruction='Test')
            model.update_obs(obs)
            result=prepare(workspace,dict(scene_update={'surfaces':{'table':dict(normal=[0,0,1],offset_m=-.75)}}))
            self.assertEqual(result['waypoints'], [])
            self.assertEqual(json.loads((workspace/'scene.json').read_text())['surfaces']['table']['offset_m'], -.75)
            # A one-axis goal remains valid even when parallel to the old closing axis.
            single_axis=prepare(workspace,dict(actions=[dict(left=dict(target=[.1,.2,.8],
                                                approach=[0,1,0]),steps=10)]))
            self.assertEqual(single_axis['waypoints'][0]['left_axes'], [[[1,0,0],[0,1,0]]])
            request=dict(scene_update={'objects':{'object':dict(shape='box',position=[.1,.2,.8],size=[.04]*3)}},
                actions=[dict(left=dict(target='object',approach=[0,0,-1],closing=[1,0,0],gripper=0),steps=20),
                         dict(left=dict(target=[.2,.2,.9]),right=dict(gripper=.5),steps=10,checkpoint='Review')])
            plan=prepare(workspace,request)
            self.assertEqual(len(plan['waypoints'][0]['left_axes']),2)
            self.assertEqual(plan['waypoints'][1]['left_axes'],[])
            self.assertFalse(plan['waypoints'][1]['right_move'])
            pose=plan['waypoints'][0]['left_pose']
            rotation=Rotation.from_quat(np.array(pose[3:])[[1,2,3,0]]).as_matrix()
            np.testing.assert_allclose(np.array(pose[:3])+rotation@np.array([.145,0,0]), [.1,.2,.8])
            second=plan['waypoints'][1]['left_pose']
            np.testing.assert_allclose(second[3:], pose[3:])
            np.testing.assert_allclose(np.array(second[:3])+rotation@np.array([.145,0,0]), [.2,.2,.9])
            updated=json.loads((workspace/'scene.json').read_text())
            self.assertEqual(updated['robots']['left']['pose'],state['left_ee_pose'])
            prepare(workspace,dict(attachments=[dict(object='object',robot='left')],finish={'status':'complete'}))
            obs['state']['left_ee_pose']=[.2,0,1,1,0,0,0]
            model.update_obs(obs)
            updated=json.loads((workspace/'scene.json').read_text())
            np.testing.assert_allclose(updated['objects']['object']['position'], [.3,.2,.8])
            obs['state']['left_ee_joint_state']=[.4]  # The object prevents full closure.
            synchronize(workspace,dict(state=obs['state'],cameras={}),
                        {'left_ee_joint_state':[0], 'right_ee_joint_state':[1]})
            continuation=prepare(workspace,dict(actions=[dict(steps=10)]))
            self.assertEqual(continuation['waypoints'][0]['left_gripper'],0)
            self.assertEqual(json.loads((workspace/'scene.json').read_text())['robots']['left']['gripper'],.4)
            with self.assertRaises(ValueError):
                prepare(workspace,dict(actions=[dict(steps=64),dict(steps=64)]))
            model.finish(obs)

if __name__=='__main__':unittest.main()
