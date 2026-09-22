"""One ordinary Codex agent and a complete skill workspace per native episode."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import time
import uuid

import numpy as np
from PIL import Image
from XPolicyLab.model_template import ModelTemplate
from .motion import MotionPlanner
from .workspace_tools import synchronize, check_budget

HERE = Path(__file__).resolve().parent


def agent_environment(codex_home):
    env = {key: value for key, value in os.environ.items()
           if not key.startswith(('CODEX_', 'SPATIAL_', 'ROBODOJO_'))}
    env.update(CODEX_HOME=str(codex_home), PYTHONPATH='')
    return env


def save(path, value):
    temp = path.with_suffix('.tmp')
    temp.write_text(json.dumps(value, indent=2, default=lambda x: x.tolist()))
    temp.replace(path)


class Model(ModelTemplate):
    def __init__(self, model_cfg):
        self.cfg = model_cfg
        if model_cfg['env_cfg_type'] != 'arx_x5' or model_cfg['action_type'] != 'joint':
            raise ValueError('ARX X5 joint trajectory actions required')
        self.root = Path(os.environ['SPATIAL_RUN_ROOT']).resolve()
        self.root.mkdir(parents=True, exist_ok=True)
        self.process = None
        self.workspace = None

    def reset(self):
        if self.process is not None and self.process.poll() is None:
            raise RuntimeError('Previous episode agent is still running')
        if self.workspace is not None and self.process is None and self.obs is None:
            return
        self.workspace = self.root / f"{self.cfg['task_name']}_{uuid.uuid4().hex[:10]}"
        self.workspace.mkdir()
        skill = HERE.parents[2] / 'skills/spatial-memory'
        target = self.workspace / '.agents/skills/spatial-memory'
        shutil.copytree(skill, target, ignore=shutil.ignore_patterns('memory.md', '__pycache__', '*.pyc'))
        shutil.copy2(HERE / 'interface.md', target / 'references/robot-interface.md')
        hashes = {str(p.relative_to(target)): hashlib.sha256(p.read_bytes()).hexdigest()
                  for p in target.rglob('*') if p.is_file()}
        save(self.workspace / 'skill_manifest.json', hashes)
        for name in ('robot.py', 'workspace_tools.py'):
            shutil.copy2(HERE / name, self.workspace / name)
        assets = Path(os.environ['ROBODOJO_ROOT']) / 'Assets/Robots/x5'
        robot = self.workspace / 'robot'
        robot.mkdir()
        for name in ('X5A.urdf', 'robot_config.yml', 'curobo.yml'):
            shutil.copy2(assets / name, robot / name)
        shutil.copytree(assets / 'meshes', robot / 'meshes')
        (self.workspace / 'bridge').mkdir()
        self.codex_home = self.workspace / '.codex-home'
        self.codex_home.mkdir()
        auth = Path(os.environ.get('CODEX_HOME', str(Path.home() / '.codex'))) / 'auth.json'
        shutil.copy2(auth, self.codex_home / 'auth.json')
        (self.codex_home / 'auth.json').chmod(0o600)
        save(self.workspace / 'runtime.json', dict(simulator='Isaac Sim', version='5.1.0.0',
             python=os.sys.executable, robot='ARX X5 dual arm', skill_sha256=hashes['SKILL.md']))
        save(self.workspace / 'config.json', self.cfg)
        subprocess.run(['git', 'init', '-q', str(self.workspace)], check=True)
        self.process = None
        self.started = time.monotonic()
        self.index = 0
        self.pending = None
        self.obs = None
        self.agent_done = False
        self.pending_actions = None

    def update_obs(self, obs):
        self.obs = obs
        folder = self.workspace / 'observations' / f'{self.index:03d}'
        folder.mkdir(parents=True, exist_ok=True)
        cameras = {}
        for name, camera in obs['vision'].items():
            path = folder / f'{name}.png'
            Image.fromarray(np.asarray(camera['color'], dtype=np.uint8)).save(path)
            cameras[name] = dict(image=str(path.relative_to(self.workspace)),
                **obs.get('camera_calibration', {}).get(name, {}))
        self.observation = dict(instruction=obs.get('instruction', obs.get('instructions', '')),
            state={k:np.asarray(v).tolist() for k,v in obs['state'].items()},
            cameras=cameras, control_budget=obs.get('control_budget'),
            terminal=bool(obs.get('terminal', False)))
        save(folder / 'observation.json', self.observation)
        save(self.workspace / 'observation.json', self.observation)
        last_command = None
        if self.pending_actions is not None:
            executed = self.observation['control_budget']['steps_used'] - self.action_start_step
            if executed > 0:
                last_command = self.pending_actions[min(executed, len(self.pending_actions)) - 1]
            self.pending_actions = None
        synchronize(self.workspace, self.observation, last_command)
        shutil.copy2(self.workspace / 'scene.json', self.workspace / f'scene_{self.index:03d}.json')
        if self.pending:
            save(self.pending, dict(observation='observation.json', cameras=cameras,
                 terminal=self.observation['terminal'], control_budget=obs.get('control_budget')))
            self.pending = None

    def _launch(self):
        env = agent_environment(self.codex_home)
        # An ordinary agent can read, compute, edit and use the complete skill.
        command = ['codex', 'exec', '--ignore-user-config', '--ignore-rules', '--model', self.cfg.get('model', 'gpt-6-astra'),
            '--sandbox', 'danger-full-access', '-c', 'approval_policy="never"',
            '-c', 'features.plugins=false',
            '-c', 'model_reasoning_effort="high"',
            '-c', 'model_provider="openai_http"',
            '-c', 'model_providers.openai_http={name="OpenAI HTTP",wire_api="responses",requires_openai_auth=true,supports_websockets=false}',
            '--cd', str(self.workspace), '--json', '--output-last-message', str(self.workspace/'final.txt')]
        for camera in self.observation['cameras'].values():
            command += ['--image', str(self.workspace / camera['image'])]
        command += ['--', '-']
        prompt = ('Read the complete spatial-memory skill and its references/robot-interface.md command contract. '
                  'Use them in this task workspace to perform the task in '
                  'observation.json through robot.py. Build and maintain your environment model, plan '
                  'actions and checkpoints, and decide when the task is complete or cannot continue. '
                  'Use the prepared commands and JSON data; do not write or modify code. '
                  f'Invoke Python scripts with the prepared interpreter {os.sys.executable}. '
                  'Submit your finish decision through robot.py; the native step limit can also end the episode.')
        save(self.workspace/'command.json', command)
        with (self.workspace/'agent.jsonl').open('w') as out, (self.workspace/'agent.stderr').open('w') as err:
            self.process = subprocess.Popen(command, cwd=self.workspace, env=env, stdout=out, stderr=err,
                                            stdin=subprocess.PIPE, text=True)
            self.process.stdin.write(prompt)
            self.process.stdin.close()

    def get_action(self):
        if self.process is None:
            self._launch()
        while True:
            if self.agent_done:
                return self._hold()
            request = self.workspace / 'bridge' / f'request_{self.index + 1:03d}.json'
            deadline = time.monotonic() + self.cfg.get('request_timeout_s', 900)
            while not request.exists():
                if (self.workspace/'done.json').exists():
                    self.agent_done = True
                    return self._hold()
                if self.process.poll() is not None:
                    self._metrics()
                    raise RuntimeError(f'Agent exited {self.process.returncode} before native task end: {self.workspace}')
                if time.monotonic() > deadline or time.monotonic()-self.started > self.cfg.get('episode_timeout_s',3600):
                    self.process.terminate()
                    raise TimeoutError(f'Agent exceeded episode/request deadline: {self.workspace}')
                time.sleep(.2)
            self.index += 1
            self.pending = request.with_name(f'reply_{self.index:03d}.json')
            try:
                plan = json.loads(request.read_text())
                scene = json.loads((self.workspace / 'scene.json').read_text())
                actions = MotionPlanner(scene, root=self.workspace, contacts=plan.get('contacts', [])).compile(
                    self.obs['state'], plan['waypoints'], self.cfg.get('max_chunk_steps',96))
                check_budget(scene, len(actions), finishing=plan.get('finishing', False))
            except (ValueError, KeyError, TypeError, RuntimeError) as error:
                save(self.pending, dict(error=str(error), observation='observation.json'))
                self.pending = None
                continue
            save(request.with_name(f'actions_{self.index:03d}.json'), actions)
            self.pending_actions = actions
            self.action_start_step = self.observation['control_budget']['steps_used']
            return actions

    def _hold(self):
        state=self.obs['state']
        commands=json.loads((self.workspace/'scene.json').read_text()).get('tool_commands', {})
        return [{**{f'{s}_arm_joint_state':np.asarray(state[f'{s}_arm_joint_state'],dtype=np.float32) for s in ('left','right')},
                 **{f'{s}_ee_joint_state':np.array([commands.get(s, state.get(f'{s}_ee_joint_state',[1])[0])],dtype=np.float32) for s in ('left','right')}}]

    def finish(self, obs):
        self.update_obs(obs)
        if self.process is not None:
            try:
                self.process.wait(timeout=180)
            except subprocess.TimeoutExpired:
                self.process.terminate()
                self.process.wait(timeout=10)
        self._metrics()
        (self.codex_home / 'auth.json').unlink(missing_ok=True)

    def _metrics(self):
        usage={}
        events=self.workspace/'agent.jsonl'
        if events.exists():
            for line in events.read_text().splitlines():
                try:event=json.loads(line)
                except json.JSONDecodeError:continue
                if event['type']=='turn.completed':
                    for key, value in event.get('usage', {}).items():
                        if isinstance(value, (int, float)):
                            usage[key] = usage.get(key, 0) + value
        save(self.workspace/'metrics.json', dict(usage=usage,action_requests=self.index,
             elapsed_s=time.monotonic()-self.started,returncode=self.process.poll() if self.process else None))

    def update_obs_batch(self, obs_list):
        if len(obs_list)!=1:raise ValueError('One environment per agent')
        self.update_obs(obs_list[0])

    def get_action_batch(self, env_idx_list=None):
        return [self.get_action()]
