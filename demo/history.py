"""Read-only index of recorded robot telemetry; no robot connection or commands."""

import json
import math
import threading

JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]
NAMES = {
    "verification": "Pick and place",
    "checkpoint_iteration": "Pick and place · refinement",
    "sequence_iteration": "Pick and place · sequence",
    "spatial_iteration": "Pick and place · exploration",
    "return_to_user_rest": "Return to rest",
    "rest_iteration": "Rest position trial",
    "A_from_scratch": "Pick and place · from scratch",
    "A_baseline": "Baseline",
    "B_calibrated": "Calibration",
}


def read_json(path, default=None):
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return default if default is not None else {}


def model_angles(position, mapping=None):
    mapping = mapping or {}
    offsets = mapping.get("joint_offsets_deg", {})
    gripper_zero, gripper_scale = mapping.get("gripper_degrees", [-14.677, 1.29275])
    if not isinstance(position, dict):
        return None
    try:
        values = [
            math.radians(
                (gripper_zero + gripper_scale * float(position[j]))
                if j == "gripper"
                else float(position[j]) + offsets.get(j, 0)
            )
            for j in JOINTS
        ]
        return values if all(math.isfinite(v) for v in values) else None
    except (KeyError, TypeError, ValueError):
        return None


def objects_from_environment(env):
    objects = []
    if env.get("cube_center") is not None:
        objects.append(
            {
                "name": "Yellow block",
                "position": env["cube_center"],
                "shape": "box",
                "size": env.get("cube_dimensions_prior", [0.022] * 3),
                "relation": env.get("attachment_status", "table"),
                "drawable": True,
            }
        )
    if env.get("cup_rim_center") is not None:
        x, y, z = env["cup_rim_center"]
        objects.append(
            {
                "name": "Cup",
                "position": [x, y, z - 0.055],
                "shape": "cup",
                "size": [0.05, 0.045, 0.11],
                "relation": "table",
                "drawable": True,
            }
        )
    return objects


class RunHistory:
    def __init__(self, root):
        self.root = root.resolve()
        self.cache = {}
        self.lock = threading.Lock()

    def paths(self):
        paths = {}
        for filename in ("events.jsonl", "replay.json"):
            for path in self.root.rglob(filename):
                if path.resolve().is_relative_to(self.root):
                    key = path.parent.relative_to(self.root).as_posix()
                    if filename == "replay.json":
                        key += "/replay"
                    paths[key] = path
        for path in self.root.rglob("scene.json"):
            if path.resolve().is_relative_to(self.root) and (path.parent / "observations").is_dir():
                paths[path.parent.relative_to(self.root).as_posix() + "/observations"] = path
        return paths

    def load(self, key):
        path = self.paths().get(key)
        if path is None:
            raise KeyError(key)
        folder = path.parent
        dependencies = [path, *folder.glob("*.json")]
        if path.name == "scene.json":
            dependencies.extend(folder.glob("observations/*/observation.json"))
            dependencies.extend(folder.glob("bridge/actions_*.json"))
        mapping_path = self.root / "robot_mapping.json"
        if mapping_path.exists():
            dependencies.append(mapping_path)
        stamp = tuple(
            sorted(
                (str(p), p.stat().st_mtime_ns, p.stat().st_size) for p in dependencies
            )
        )
        with self.lock:
            if key in self.cache and self.cache[key][0] == stamp:
                return self.cache[key][1]
            if path.name == "scene.json":
                from scene_view import read_observations
                result = read_observations(path, key)
            elif path.name == "replay.json":
                from scene_view import read_replay
                result = read_replay(path, key)
            else:
                result = self.parse(key, path)
            self.cache[key] = (stamp, result)
            return result

    def parse(self, key, path):
        folder = path.parent
        mapping = read_json(
            folder / "robot_mapping.json", read_json(self.root / "robot_mapping.json")
        )
        samples, motions = [], []
        with path.open() as stream:
            for line in stream:
                # Serial packet bytes are retained on disk, not sent to the browser.
                if '"serial_packet"' in line:
                    continue
                try:
                    row = json.loads(line)
                    if row.get("event") == "monitor":
                        sample = row.get("sample", {})
                        q = model_angles(sample.get("position_deg"), mapping)
                        t = float(sample.get("time", row["time"]))
                        if q is not None and math.isfinite(t):
                            samples.append([t, *q])
                    elif row.get("event") == "move_start":
                        motions.append(
                            {"name": "Move", "start": row["time"], "end": row["time"]}
                        )
                    elif row.get("event") == "move_complete" and motions:
                        motions[-1]["end"] = row["time"]
                except (ValueError, TypeError, KeyError):
                    continue  # A writer may still be appending its last line.
        if not samples or not motions:
            return None
        samples.sort(key=lambda s: s[0])
        result = read_json(folder / "result.json")
        timing = read_json(folder / "timing.json")
        records = read_json(folder / "execution_records.json")
        if isinstance(records, list):
            records = {"records": records}
        start = min(
            samples[0][0],
            timing.get("task_started")
            or result.get("task_started")
            or result.get("started")
            or samples[0][0],
        )
        end = max(
            samples[-1][0],
            result.get("result_confirmed") or result.get("finished") or samples[-1][0],
        )
        # Include measured endpoint/observation poses, including stationary review periods.
        for state_path in folder.glob("*.json"):
            if not state_path.name.startswith(
                ("task_", "before", "arrived", "torque_off")
            ):
                continue
            state = read_json(state_path)
            q = model_angles(state.get("position_deg"), mapping)
            t = state.get("time")
            if q is not None and isinstance(t, (int, float)) and start <= t <= end:
                samples.append([t, *q])
        samples = list(
            {row[0]: row for row in sorted(samples, key=lambda s: s[0])}.values()
        )
        actions = []
        for field in (
            "observation_motion_records",
            "push_records",
            "records",
            "rest_records",
        ):
            actions.extend(records.get(field, []))
        actions = sorted(actions or motions, key=lambda a: a["start"])
        actions = [
            {
                "name": a["name"].replace("_", " ").capitalize(),
                "start": max(0, a["start"] - start),
                "end": a["end"] - start,
            }
            for a in actions
        ]
        checkpoints = []
        for field in (
            "observation_checkpoints",
            "push_checkpoints",
            "checkpoint_records",
        ):
            for c in records.get(field, []):
                if c.get("arrived_at") is not None:
                    checkpoints.append(
                        {
                            "name": c["after_action"].replace("_", " ").capitalize(),
                            "time": c["arrived_at"] - start,
                        }
                    )
        env = read_json(folder / "plan.json").get("environment", {})
        scene_states = [{"time": 0, "objects": objects_from_environment(env)}]
        # Replans are committed at the corresponding recorded review.
        replans = sorted(
            folder.glob("replan_*.json"), key=lambda p: int(p.stem.split("_")[-1])
        )
        updates = [
            r
            for r in records.get("reviews", [])
            if r.get("cube_pixel") is not None or r.get("cup_pixel") is not None
        ]
        for review, replan in zip(updates, replans):
            updated_env = read_json(replan).get("environment", {})
            scene_states.append(
                {
                    "time": review["time"] - start,
                    "objects": objects_from_environment(updated_env),
                }
            )
        for review in records.get("reviews", []):
            if review.get("decision") in ("attached", "retry", "released"):
                moment = review["time"] - start
                if review["decision"] in ("attached", "released"):
                    action_name = (
                        "Close" if review["decision"] == "attached" else "Release"
                    )
                    preceding = [
                        a
                        for a in actions
                        if a["name"] == action_name and a["end"] <= moment
                    ]
                    if preceding:
                        moment = preceding[-1]["end"]
                scene_states.append({"time": moment, "attachment": review["decision"]})
        for sample in samples:
            sample[0] -= start
        title = next(
            (
                title
                for prefix, title in NAMES.items()
                if folder.name.startswith(prefix)
            ),
            folder.name.replace("_", " "),
        )
        completed = result.get("success") is True or result.get("status") == "complete"
        summary = {
            "id": key,
            "title": title,
            "started_at": start,
            "duration": end - start,
            "samples": len(samples),
            "actions": len(actions),
            "status": "Completed" if completed else "Recorded",
        }
        return {
            "summary": summary,
            "joints": JOINTS,
            "samples": samples,
            "actions": actions,
            "checkpoints": sorted(checkpoints, key=lambda c: c["time"]),
            "scene_states": sorted(scene_states, key=lambda s: s["time"]),
            "plane": [0, 0, 1, -env["table_z"]] if "table_z" in env else None,
            "attachment_local": env.get("actual_contact_geometry", {}).get(
                "attachment_local_prior_m", [-0.008, 0, 0.002]
            ),
            "tip_link": "gripper_frame_link",
        }

    def index(self):
        summaries = []
        for key in self.paths():
            try:
                run = self.load(key)
                if run:
                    summaries.append(run["summary"])
            except (OSError, ValueError, KeyError):
                continue
        return sorted(summaries, key=lambda r: r["started_at"], reverse=True)
