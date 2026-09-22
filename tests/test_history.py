"""Offline tests for replay provenance, ordering, and incomplete recordings."""

import json
import math
from pathlib import Path
import tempfile
import unittest
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "demo"))
from history import JOINTS, RunHistory, model_angles


class HistoryTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.folder = self.root / "trial"
        self.folder.mkdir()
        self.events = self.folder / "events.jsonl"
        self.position = dict.fromkeys(JOINTS, 0)
        self.history = RunHistory(self.root)

    def write(self, rows):
        self.events.write_text("".join(json.dumps(row) + "\n" for row in rows))

    def monitor(self, t, position=None):
        return {
            "event": "monitor",
            "time": t,
            "sample": {"position_deg": position or self.position, "time": t},
        }

    def moving(self):
        self.write(
            [
                {"event": "move_start", "time": 10},
                self.monitor(10),
                self.monitor(11),
                {"event": "move_complete", "time": 11},
            ]
        )

    def test_calibrated_pose_mapping(self):
        q = model_angles(self.position, {"joint_offsets_deg": {"shoulder_lift": 4.159}})
        self.assertAlmostEqual(q[1], math.radians(4.159))
        self.assertAlmostEqual(q[-1], math.radians(-14.677))
        self.assertIsNone(model_angles({"shoulder_pan": 3}))
        self.assertIsNone(model_angles({**self.position, "gripper": float("nan")}))

    def test_only_measured_motion_becomes_replay(self):
        self.write([self.monitor(10)])
        self.assertEqual(self.history.index(), [])
        self.moving()
        self.assertEqual(len(self.history.index()), 1)
        run = self.history.load("trial")
        self.assertEqual([s[0] for s in run["samples"]], [0, 1])
        self.assertEqual(run["summary"]["status"], "Recorded")
        with self.assertRaises(KeyError):
            self.history.load("../trial")

    def test_partial_line_and_cache_refresh(self):
        self.moving()
        self.assertEqual(self.history.load("trial")["summary"]["samples"], 2)
        with self.events.open("a") as stream:
            stream.write(json.dumps(self.monitor(12)) + '\n{"event":')
        self.assertEqual(self.history.load("trial")["summary"]["samples"], 3)

    def test_stationary_time_and_confirmation_are_preserved(self):
        self.moving()
        (self.folder / "result.json").write_text(
            json.dumps({"success": True, "task_started": 5, "result_confirmed": 15})
        )
        (self.folder / "execution_records.json").write_text(
            json.dumps(
                {
                    "records": [{"name": "close", "start": 10, "end": 11}],
                    "reviews": [{"decision": "attached", "time": 14}],
                }
            )
        )
        run = self.history.load("trial")
        self.assertEqual(run["summary"]["duration"], 10)
        self.assertEqual(run["samples"][0][0], 5)
        self.assertEqual(run["scene_states"][-1], {"time": 6, "attachment": "attached"})
        self.assertEqual(run["summary"]["status"], "Completed")

    def test_tool_frame_exists_in_robot_model(self):
        import sys

        skill = Path(__file__).resolve().parents[1] / "skills/spatial-memory"
        sys.path.insert(0, str(skill / "scripts"))
        from kinematics import Robot

        robot = Robot(skill / "assets/so101/robot.urdf")
        self.moving()
        self.assertIn(self.history.load("trial")["tip_link"], robot.links)

    def test_older_list_records(self):
        self.moving()
        (self.folder / "execution_records.json").write_text(
            json.dumps([{"name": "return", "start": 10, "end": 11}])
        )
        self.assertEqual(self.history.load("trial")["actions"][0]["name"], "Return")


if __name__ == "__main__":
    unittest.main()
