"""Offline trajectory and checkpoint behavior; no device connections."""

import sys
from pathlib import Path
import unittest
import numpy as np

sys.path.insert(
    0, str(Path(__file__).resolve().parents[1] / "skills/spatial-memory/scripts")
)
from execution import ActionSequence
from trajectory import WaypointTrajectory


class TrajectoryTests(unittest.TestCase):
    def test_passes_knots_without_stopping_and_respects_speed(self):
        path = WaypointTrajectory({"x": 0}, [{"x": 10}, {"x": 20}], speed=6)
        np.testing.assert_allclose(path.curve(path.times).ravel(), [0, 10, 20])
        self.assertGreater(path.at(path.times[1], 1)["x"], 0)
        self.assertAlmostEqual(path.at(0, 1)["x"], 0)
        self.assertAlmostEqual(path.at(path.duration, 1)["x"], 0)
        self.assertLessEqual(path.peak_speed.max(), 6 + 1e-9)
        epsilon = 1e-7
        self.assertAlmostEqual(
            path.at(path.times[1] - epsilon, 1)["x"],
            path.at(path.times[1] + epsilon, 1)["x"],
            places=5,
        )

    def test_no_overshoot_at_turns_or_constant_axes(self):
        path = WaypointTrajectory(
            {"x": 0, "y": 7},
            [{"x": 4}, {"x": 2}, {"x": 6}],
            speed=3,
            limits={"x": (0, 6), "y": (7, 7)},
        )
        for i, (a, b) in enumerate(zip(path.times, path.times[1:])):
            values = path.curve(np.linspace(a, b, 201))
            lo, hi = (
                np.minimum(path.points[i], path.points[i + 1]),
                np.maximum(path.points[i], path.points[i + 1]),
            )
            self.assertTrue(np.all(values >= lo - 1e-9))
            self.assertTrue(np.all(values <= hi + 1e-9))

    def test_bounds_and_time_scaling(self):
        path = WaypointTrajectory({"x": 0}, [{"x": 9}], speed=6, max_acceleration=2)
        self.assertLessEqual(path.peak_acceleration.max(), 2 + 1e-9)
        with self.assertRaises(ValueError):
            WaypointTrajectory({"x": 0}, [{"x": 11}], speed=6, limits={"x": (0, 10)})
        with self.assertRaises(ValueError):
            WaypointTrajectory({"x": 0}, [{"x": float("nan")}], speed=6)
        with self.assertRaises(ValueError):
            path.samples(0)


class Controller:
    limits = {"arm": (-100, 100), "gripper": (0, 100)}

    def __init__(self):
        self.q = {"arm": 0, "gripper": 20}
        self.curves, self.moves, self.held = [], [], False

    def read(self):
        return self.q.copy()

    def move(self, target, **kwargs):
        self.moves.append(target.copy())
        self.q = target.copy()
        # The object blocks further closing; the commanded grasp must persist.
        self.q["gripper"] = max(10, self.q["gripper"])
        return {"position_deg": self.read()}

    def move_trajectory(self, trajectory, *, on_waypoint, **kwargs):
        self.curves.append(trajectory)
        for index, t in enumerate(trajectory.times[1:]):
            self.q = trajectory.at(t)
            on_waypoint(index, {"position_deg": self.read()})
        return {"position_deg": self.read()}

    def hold(self):
        self.held = True


class SequenceTests(unittest.TestCase):
    def test_checkpoints_tool_actions_and_grasp_persistence(self):
        controller = Controller()
        observations, checked = [], []
        actions = [
            {"name": "via", "target": {"arm": 10}},
            {
                "name": "near",
                "target": {"arm": 20},
                "checkpoint": {"decision": "Align"},
            },
            {"name": "engage", "target": {"arm": 25}},
            {"name": "close", "target": {"gripper": 7}},
            {"name": "lift", "target": {"arm": 15}},
            {
                "name": "carry",
                "target": {"arm": 5},
                "checkpoint": {"decision": "Release?"},
            },
        ]
        sequence = ActionSequence(
            controller,
            actions,
            observe=lambda a: observations.append(a["name"]),
            tool_channels=["gripper"],
            trajectory_validator=lambda t, a: checked.append(t),
        )
        self.assertEqual(sequence.run()["status"], "checkpoint")
        self.assertEqual(observations, ["near"])
        self.assertEqual(len(controller.curves), 1)
        self.assertGreater(
            controller.curves[0].at(controller.curves[0].times[1], 1)["arm"], 0
        )
        self.assertEqual(sequence.resume()["status"], "checkpoint")
        self.assertEqual(observations, ["near", "carry"])
        self.assertEqual(len(controller.moves), 1)
        self.assertEqual(controller.curves[-1].at(0)["gripper"], 7)
        self.assertEqual(len(sequence.records), 6)
        self.assertFalse(sequence.records[0]["stopped"])
        self.assertEqual(sequence.resume()["status"], "complete")
        self.assertEqual(len(checked), len(controller.curves))

    def test_explicit_stop_keeps_execution_in_one_agent_call(self):
        controller = Controller()
        sequence = ActionSequence(
            controller,
            [{"target": {"arm": 10}, "stop": True}, {"target": {"arm": 20}}],
            observe=lambda _: self.fail("Unexpected observation"),
            trajectory_validator=lambda *_: None,
        )
        self.assertEqual(sequence.run()["status"], "complete")
        self.assertEqual(len(controller.curves), 2)

    def test_rejected_curve_never_reaches_controller(self):
        controller = Controller()

        def reject(*_):
            raise ValueError("Path intersects object")

        sequence = ActionSequence(
            controller,
            [{"target": {"arm": 10}}],
            observe=lambda _: None,
            trajectory_validator=reject,
        )
        with self.assertRaisesRegex(ValueError, "intersects"):
            sequence.run()
        self.assertEqual(controller.curves, [])
        self.assertTrue(controller.held)

    def test_segmented_adapter_remains_supported(self):
        controller = Controller()
        sequence = ActionSequence(
            controller, [{"target": {"arm": 60}}], observe=lambda _: None, max_delta=45
        )
        self.assertEqual(sequence.run()["status"], "complete")
        self.assertEqual(len(controller.moves), 2)
        self.assertEqual(controller.curves, [])


if __name__ == "__main__":
    unittest.main()
