#!/usr/bin/env python3
"""Report native scores, agent usage and execution time."""
import argparse
import json
from pathlib import Path
import statistics
from run_io import write


def audit(workspace):
    commands=[];usage_updates=set();events=[]
    p=workspace/'agent.jsonl'
    if p.exists():
        for line in p.read_text().splitlines():
            try:event=json.loads(line)
            except json.JSONDecodeError:continue
            events.append(event)
            item=event.get('item',{})
            if event['type']=='item.completed' and item.get('type')=='command_execution':commands.append(item['command'])
    for path in (workspace/'.codex-home/sessions').rglob('*.jsonl'):
        for line in path.read_text().splitlines():
            try:payload=json.loads(line).get('payload',{})
            except json.JSONDecodeError:continue
            if payload.get('type')=='token_count' and payload.get('info'):
                u=payload['info']['total_token_usage']
                usage_updates.add(tuple(u.get(k,0) for k in ('input_tokens','cached_input_tokens','output_tokens')))
    modeling_rounds = 0
    for job in (workspace/'.asset_jobs').glob('*/job.json'):
        worker = Path(json.loads(job.read_text())['workspace'])
        updates = set()
        for path in (worker/'.codex-home/sessions').rglob('*.jsonl'):
            for line in path.read_text().splitlines():
                try: payload = json.loads(line).get('payload', {})
                except json.JSONDecodeError: continue
                if payload.get('type') == 'token_count' and payload.get('info'):
                    u = payload['info']['total_token_usage']
                    updates.add(tuple(u.get(k, 0) for k in ('input_tokens','cached_input_tokens','output_tokens')))
        modeling_rounds += len(updates)
    scene_paths={p for pattern in ('scene.*','environment.*','world.*','scene/**/*','environment/**/*')
                 for p in workspace.glob(pattern) if p.is_file() and p.suffix in ('.json','.xml','.mjcf','.usd','.usda','.yaml')}
    return dict(workspace=str(workspace),shell_calls=len(commands),
        reported_inference_rounds=len(usage_updates)+modeling_rounds,
        task_inference_rounds=len(usage_updates),modeling_inference_rounds=modeling_rounds,
        skill_read=any('SKILL.md' in c and ('.agents/skills' in c or 'spatial-memory' in c) for c in commands),
        geometry_commands=[c for c in commands if any(x in c for x in
            ('triangulate','pixel_plane','pixel-plane','kinematics','geometry.py',
             'scene.py locate','scene.py project','scene.py pose','scene.py ik'))],
        scene_files=sorted(str(p.relative_to(workspace)) for p in scene_paths),commands=commands)


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('run',type=Path);args=parser.parse_args()
    manifest=json.loads((args.run/'manifest.json').read_text());rows=[]
    for task,entry in manifest['tasks'].items():
        agents=[]
        for p in (args.run/task/'policy').glob('*/metrics.json'):
            metric=json.loads(p.read_text());agents.append(dict(**metric,audit=audit(p.parent)))
        usage={k:sum((a.get('usage') or {}).get(k,0) for a in agents) for k in ('input_tokens','cached_input_tokens','output_tokens')}
        execution=args.run/task/'policy/execution.jsonl'
        timing=[json.loads(line) for line in execution.read_text().splitlines()] if execution.exists() else []
        rows.append(dict(task=task,status=entry['status'],details=entry.get('details',[]),agents=agents,usage=usage,
             native_results=entry.get('native_results',[]),wall_s=sum(a.get('elapsed_s',0) for a in entry['attempts']),
             policy_wait_s=sum(t['policy_call_s'] for t in timing),execution_s=sum(t['execution_s'] for t in timing)))
    details=[d for r in rows for d in r['details']]
    total_usage={k:sum(r['usage'][k] for r in rows) for k in ('input_tokens','cached_input_tokens','output_tokens')}
    totals=dict(completed=len(details),successes=sum(d['success'] for d in details),
        mean_score=statistics.mean(d['score'] for d in details) if details else None,
        wall_s=sum(r['wall_s'] for r in rows),usage=total_usage)
    totals['reported_tokens']=total_usage['input_tokens']+total_usage['output_tokens']
    totals['uncached_input_plus_output']=totals['reported_tokens']-total_usage['cached_input_tokens']
    report=dict(version_sha256=manifest['version_sha256'],seed=manifest['seed'],tasks=rows,totals=totals)
    write(args.run/'report.json',report)
    lines=['# Complete spatial-memory skill: small local evaluation','',
        f"Configuration seed {manifest['seed']}; one native layout per selected task.",'',
        '| Task | Native success | Score / 100 | Wall min | Input tokens | Cached input | Output tokens |',
        '| --- | ---: | ---: | ---: | ---: | ---: | ---: |']
    for r in rows:
        success=str(sum(d['success'] for d in r['details']))+'/'+str(len(r['details']))
        score=f"{100*statistics.mean(d['score'] for d in r['details']):.1f}" if r['details'] else 'pending'
        u=r['usage'];lines.append(f"| {r['task']} | {success} | {score} | {r['wall_s']/60:.1f} | {u['input_tokens']:,} | {u['cached_input_tokens']:,} | {u['output_tokens']:,} |")
    lines+=['','Cached input is included in input tokens. All counts are reported usage, not dollar charges.',
        'One CLI agent turn includes multiple model/tool rounds. Action chunks are not model-call counts.',
        'Policy usage includes task agents and their asset-modeling workers. Startup failures are retained separately.', '',
        '## Full-skill evidence','',
        '| Task | Skill read | Geometry-related commands | Shell calls | Reported inference rounds | Action requests | Scene files |',
        '| --- | --- | ---: | ---: | ---: | ---: | --- |']
    for r in rows:
        for a in r['agents']:
            x=a['audit'];lines.append(f"| {r['task']} | {x['skill_read']} | {len(x['geometry_commands'])} | {x['shell_calls']} | {x['reported_inference_rounds']} | {a['action_requests']} | {', '.join(x['scene_files'])} |")
    lines+=['','## Timing','',
        '| Task | Policy wait min | Simulator execution min | Startup / shutdown / other min |',
        '| --- | ---: | ---: | ---: |']
    for r in rows:
        lines.append(f"| {r['task']} | {r['policy_wait_s']/60:.2f} | {r['execution_s']/60:.2f} | {(r['wall_s']-r['policy_wait_s']-r['execution_s'])/60:.2f} |")
    lines+=['','Policy wait includes model inference, observation review, geometry tools, scene updates and planning. Those activities share a persistent agent session; the logs do not reliably separate their individual token costs.']
    lines+=['','The complete skill was copied into each episode workspace, with per-file hashes. Geometry calculations and scene files were produced by the task agent. The wrapper supplied observations and executed submitted waypoints.',
        'Inputs include calibrated camera matrices as well as RGB and robot state; no object ground-truth pose or evaluator reward is supplied. Task input access must also be assessed from the retained command audit.',
        'Isaac Sim executes the benchmark. Agent scene exports are separate estimated geometry; an MJCF export is not evidence of an additional live dynamics simulation.']
    (args.run/'report.md').write_text('\n'.join(lines)+'\n')
    print(json.dumps(totals))

if __name__=='__main__':main()
