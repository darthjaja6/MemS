#!/usr/bin/env python3
"""Run a small, resumable evaluation of complete task-workspace agents."""
import argparse
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time
from run_io import write

TASKS=['stack_blocks','hang_mugs','insert_tubes','organize_table','swap_blocks','classify_objects_by_language']


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--tasks',nargs='+',default=TASKS)
    parser.add_argument('--seed',type=int,default=2)
    args=parser.parse_args()
    root=Path(os.environ['ROBODOJO_ROOT']).resolve()
    env_path=os.environ.get('VIRTUAL_ENV') or os.environ.get('CONDA_PREFIX')
    if not env_path:raise ValueError('Activate the working RoboDojo virtualenv or conda environment')
    env=Path(env_path).resolve()
    out=args.output.resolve();out.mkdir(parents=True,exist_ok=True)
    adapter=Path(__file__).parent/'astra_agent'
    skill=Path(__file__).resolve().parents[2]/'skills/spatial-memory'
    digest=hashlib.sha256()
    for base in [adapter,skill]:
        for path in sorted(base.rglob('*')):
            if path.is_file() and '__pycache__' not in path.parts and path.name!='memory.md':
                if base == adapter and path.name == 'README.md':continue
                digest.update(str(path.relative_to(base)).encode()+path.read_bytes())
    manifest_path=out/'manifest.json'
    manifest=json.loads(manifest_path.read_text()) if manifest_path.exists() else dict(
        created_at=time.time(),version_sha256=digest.hexdigest(),seed=args.seed,tasks={})
    if (manifest['version_sha256'],manifest['seed'])!=(digest.hexdigest(),args.seed):
        raise ValueError('Use a new output directory after policy/skill/seed changes')
    for task in args.tasks:
        entry=manifest['tasks'].setdefault(task,{})
        if entry.get('status')=='complete':continue
        folder=out/task;folder.mkdir(exist_ok=True)
        run_id=entry.setdefault('run_id',datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))
        command=['bash',str(root/'scripts/robodojo.sh'),'eval','--task',task,
            '--policy-dir',str(root/'XPolicyLab/policy/astra_agent'),'--ckpt','codex-full-skill',
            '--env-cfg','arx_x5','--action-type','joint','--seed',str(args.seed),
            '--policy-gpu','0','--env-gpu','0','--policy-env',str(env),'--eval-env',str(env),'--eval-num','1']
        attempt=dict(started_at=time.time());entry.setdefault('attempts',[]).append(attempt)
        entry.update(status='running',command=command);write(manifest_path,manifest)
        log_path=folder/f'attempt_{len(entry["attempts"]):02d}.log'
        process_env=dict(os.environ,SPATIAL_RUN_ROOT=str(folder/'policy'),ROBODOJO_RUN_ID=run_id,
                         ROBODOJO_MAX_BASH_RETRIES='1',PYTHONUNBUFFERED='1')
        print('START',task,flush=True)
        with log_path.open('w') as log:
            process=subprocess.Popen(command,cwd=root,env=process_env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            try:
                while True:
                    try:
                        attempt['returncode']=process.wait(timeout=15);break
                    except subprocess.TimeoutExpired:
                        with log_path.open('rb') as stream:
                            stream.seek(max(0,log_path.stat().st_size-16000))
                            tail=stream.read().decode(errors='replace')
                        if 'Policy request failed:' in tail:
                            raise RuntimeError('Policy request failed; preserve attempt and stop stalled simulator')
                        if time.time()-attempt['started_at']>4200:raise TimeoutError('Task wall deadline')
            except (RuntimeError,TimeoutError,KeyboardInterrupt) as error:
                attempt['error']=str(error)
                os.killpg(process.pid,signal.SIGTERM)
                try:process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGKILL);process.wait()
                # Kit can outlive its shell after SIGTERM; stop the entire trial.
                try:os.killpg(process.pid,signal.SIGKILL)
                except ProcessLookupError:pass
                attempt['returncode']=process.returncode
        attempt['elapsed_s']=time.time()-attempt['started_at']
        for auth in (folder/'policy').glob('*/.codex-home/auth.json'):
            auth.unlink(missing_ok=True)
        native=list((root/'eval_result/RoboDojo'/task/'astra_agent/arx_x5').glob(f'*/{run_id}/_result.json'))
        entry['native_results']=[str(p) for p in native]
        entry['details']=[d for p in native for d in json.loads(p.read_text()).get('details',{}).values()]
        entry['agent_metrics']=[dict(workspace=str(p.parent),**json.loads(p.read_text())) for p in folder.glob('policy/*/metrics.json')]
        entry['status']='complete' if len(entry['details'])==1 else 'incomplete'
        write(manifest_path,manifest)
        print('END',task,entry['status'],entry['details'],flush=True)
        if entry['status']!='complete':break

if __name__=='__main__':main()
