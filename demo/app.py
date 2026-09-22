#!/usr/bin/env python3
"""Read-only spatial memory viewer. Never connects to or commands hardware."""

import argparse
import hashlib
from datetime import datetime
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import mimetypes
from pathlib import Path
import sys
from urllib.parse import urlparse, unquote, parse_qs

HERE = Path(__file__).resolve().parent
SKILL = HERE.parent / "skills/spatial-memory"
sys.path.insert(0, str(SKILL / "scripts"))
from kinematics import Robot
from history import RunHistory


def table_rows(text):
    fenced = False
    for line in text.splitlines():
        if line.lstrip().startswith("```"):
            fenced = not fenced
        if not fenced and line.strip().startswith("|"):
            yield [v.strip() for v in line.strip().strip("|").split("|")]


def numbers(values):
    try:
        result = [float(v) for v in values]
        return result if all(math.isfinite(v) for v in result) else None
    except ValueError:
        return None


class MemoryView:
    def __init__(self, path):
        self.path = path.resolve()
        self.robot = Robot(SKILL / "assets/so101/robot.urdf")

    def state(self):
        # Read fresh on every request; this process never writes memory.
        with self.path.open(encoding="utf-8") as source:
            import os

            modified = os.fstat(source.fileno()).st_mtime
            text = source.read()
        objects, q = [], {}
        plane = None
        for row in table_rows(text):
            if len(row) == 6 and row[0] not in ("Object", "---"):
                position = numbers(row[1:4])
                parts = row[4].split()
                shape = parts[0] if parts else ""
                size = numbers(parts[1:])
                drawable = (
                    position is not None
                    and shape in ("box", "cup", "ellipsoid")
                    and size is not None
                    and len(size) == 3
                    and min(size) > 0
                )
                if shape == "cup" and drawable:
                    drawable = size[0] > size[1]
                objects.append(
                    {
                        "name": row[0],
                        "position": position,
                        "shape": shape,
                        "size": size,
                        "relation": row[5],
                        "drawable": drawable,
                    }
                )
            elif len(row) == 2 and row[0] in self.robot.joints:
                value = numbers(row[1:])
                if value is not None:
                    q[row[0]] = value[0]
            elif len(row) == 5 and row[0] == "table":
                values = numbers(row[1:])
                if values and math.sqrt(sum(v * v for v in values[:3])) > 1e-8:
                    plane = values
        required = [
            name for name, j in self.robot.joints.items() if j["type"] != "fixed"
        ]
        pose_known = all(name in q for name in required)
        body_pose_known = all(name in q for name in required if name != "gripper")
        frames = self.robot.frames(q if body_pose_known else {})
        known_links = {
            link: body_pose_known
            and all(
                j["name"] in q for j in self.robot.chain(link) if j["type"] != "fixed"
            )
            for link in self.robot.links
        }
        return {
            "revision": hashlib.sha256(text.encode()).hexdigest(),
            "source": str(self.path),
            "updated_at": datetime.fromtimestamp(modified).astimezone().isoformat(),
            "objects": objects,
            "plane": plane,
            "pose_known": pose_known,
            "body_pose_known": body_pose_known,
            "known_links": known_links,
            "frames": {name: T.tolist() for name, T in frames.items()},
        }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--memory", type=Path, help="Legacy Markdown scene")
    parser.add_argument("--scene", type=Path, help="Live scene.json (takes precedence over --memory)")
    parser.add_argument("--runs", type=Path, help="Directory of recorded sessions")
    args = parser.parse_args()
    example = not args.scene and not args.memory
    if example:
        args.scene = HERE.parent / "examples/stacking/scene.json"
    if args.scene:
        from scene_view import SceneView
        view = SceneView(args.scene)
    else:
        view = MemoryView(args.memory)
    history = RunHistory(args.runs or (HERE.parent / "examples" if example else HERE.parent / "runs"))

    class Handler(SimpleHTTPRequestHandler):
        def send(self, data, kind="application/json", status=200):
            if not isinstance(data, bytes):
                data = (
                    json.dumps(data, ensure_ascii=False, allow_nan=False)
                    if kind == "application/json"
                    else data
                ).encode()
            self.send_response(status)
            self.send_header(
                "Content-Type",
                kind + "; charset=utf-8"
                if kind.startswith("text/") or kind == "application/json"
                else kind,
            )
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            path = urlparse(self.path).path
            try:
                if path == "/api/runs":
                    return self.send(history.index())
                if path.startswith("/api/runs/"):
                    try:
                        run = history.load(unquote(path[len("/api/runs/") :]))
                    except KeyError:
                        run = None
                    return self.send(
                        run or {"error": "Run not found"}, status=200 if run else 404
                    )
                if path == "/api/state":
                    if isinstance(view, MemoryView):
                        return self.send(view.state())
                    since = parse_qs(urlparse(self.path).query).get('since', [None])[0]
                    return self.send(view.state(since))
                if path == "/api/legacy-robot":
                    return self.send(Robot(SKILL / "assets/so101/robot.urdf").description())
                if path == "/api/robot":
                    return self.send(view.robot.description() if isinstance(view, MemoryView) else {"generic": True})
                base, relative = (
                    (SKILL / "assets/so101", path[len("/robot/") :])
                    if path.startswith("/robot/")
                    else (HERE, path.lstrip("/") or "index.html")
                )
                file = (base / relative).resolve()
                if not file.is_relative_to(base.resolve()) or not file.is_file():
                    return self.send({"error": "Not found"}, status=404)
                return self.send(
                    file.read_bytes(),
                    mimetypes.guess_type(file.name)[0] or "application/octet-stream",
                )
            except (OSError, ValueError) as error:
                return self.send({"error": str(error)}, status=503)

        def do_POST(self):
            self.send({"error": "Read-only viewer"}, status=405)

        def log_message(self, *_):
            pass

    server = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    print(
        f"MemS: http://127.0.0.1:{args.port}\nReading: {view.path}",
        flush=True,
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        server.server_close()


if __name__ == "__main__":
    main()
