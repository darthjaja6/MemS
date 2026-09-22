#!/usr/bin/env python3
"""Submit a JSON scene update and action sequence, or an agent finish decision."""
import argparse
import json
from pathlib import Path
import time
from workspace_tools import prepare


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path)
    parser.add_argument('--preview', action='store_true', help='Save the scene and plan without moving')
    args = parser.parse_args()
    root = Path(__file__).resolve().parent
    request = json.loads(args.input.read_text())
    plan = prepare(root, request)
    if args.preview:
        print(json.dumps(plan))
        return
    if plan['waypoints']:
        bridge = root / 'bridge'
        index = len(list(bridge.glob('request_*.json'))) + 1
        path = bridge / f'request_{index:03d}.json'
        temp = path.with_suffix('.tmp')
        temp.write_text(json.dumps(plan))
        temp.replace(path)
        reply = bridge / f'reply_{index:03d}.json'
        while not reply.exists():
            time.sleep(.2)
        result = json.loads(reply.read_text())
        print(json.dumps(result))
        if 'error' in result:
            raise SystemExit(1)
    elif not request.get('finish'):
        print(json.dumps({'scene': 'scene.json'}))
    if request.get('finish'):
        (root / 'done.json').write_text(json.dumps(request['finish'], indent=2))
        print(json.dumps({'agent_finished': request['finish']}))


if __name__ == '__main__':
    main()
