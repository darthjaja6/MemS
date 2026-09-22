"""Execute a planned action sequence through a protected robot adapter.

Adapter: read() -> joint mapping; move(target, speed, max_delta) -> state;
hold(). Positions and speeds use the adapter's native units. The caller owns
observation and scene publication callbacks; this module owns no hardware.
"""

import math
import time


class ActionSequence:
    def __init__(
        self,
        controller,
        actions,
        *,
        observe,
        publish=None,
        max_delta=45.0,
        speed=5.0,
        tool_channels=(),
        expected_start=None,
        start_tolerance=None,
        trajectory_validator=None,
    ):
        self.controller = controller
        self.trajectory_validator = trajectory_validator
        if trajectory_validator is not None and not callable(
            getattr(controller, "move_trajectory", None)
        ):
            raise ValueError(
                "Continuous execution requires move_trajectory on the controller"
            )
        self.observe = observe
        self.publish = publish or (lambda action, state: None)
        self.max_delta = float(max_delta)
        self.speed = float(speed)
        if not math.isfinite(self.max_delta) or self.max_delta <= 0:
            raise ValueError("max_delta must be positive and finite")
        if not math.isfinite(self.speed) or self.speed <= 0:
            raise ValueError("speed must be positive and finite")
        self.tool_channels = set(tool_channels)
        self.expected_start = expected_start
        self.start_tolerance = start_tolerance or {}
        self.targets = dict(controller.read())
        if not self.tool_channels <= self.targets.keys():
            raise ValueError("Tool channels must exist in the controller")
        self.actions = self._validate(actions)
        self.index = 0
        self.waiting = False
        self.failed = False
        self.started_at = None
        self.records = []
        self.checkpoints = []

    def _validate(self, actions):
        checked = []
        for action in actions:
            action = {**action, "target": dict(action["target"])}
            if not action["target"] or not set(action["target"]) <= self.targets.keys():
                raise ValueError("Action targets must name known controller channels")
            if not all(math.isfinite(v) for v in action["target"].values()):
                raise ValueError("Action targets must be finite")
            for joint, value in action["target"].items():
                limits = getattr(self.controller, "limits", {}).get(joint)
                if limits is not None:
                    lo, hi = limits
                    if (
                        not min(lo, self.targets[joint])
                        <= value
                        <= max(hi, self.targets[joint])
                    ):
                        raise ValueError(f"Action exceeds controller limits: {joint}")
            speed = action.get("speed", self.speed)
            if not math.isfinite(speed) or speed <= 0:
                raise ValueError("Action speed must be positive and finite")
            if "checkpoint" in action and not action["checkpoint"].get("decision"):
                raise ValueError("Each checkpoint needs a decision")
            checked.append(action)
        return checked

    def resume(self, remaining_actions=None, *, prepare=None):
        """Apply a review and continue. prepare() may return revised remaining actions."""
        if self.failed:
            raise RuntimeError(
                "Execution failed; replan from the current measured state"
            )
        if not self.waiting:
            raise RuntimeError("No checkpoint is waiting for review")
        if remaining_actions is not None and prepare is not None:
            raise ValueError("Pass revised actions directly or through prepare")
        checkpoint = self.checkpoints[-1]
        checkpoint["review_received_at"] = time.time()
        if prepare is not None:
            remaining_actions = prepare()
        if remaining_actions is not None:
            self.actions[self.index :] = self._validate(remaining_actions)
        checkpoint["resumed_at"] = time.time()
        self.waiting = False
        return self.run()

    def _motion_group(self):
        """Tool-only operations separate body paths without requiring a review."""
        group, previous = [], self.targets.copy()
        for action in self.actions[self.index :]:
            target = {**previous, **action["target"]}
            body_changed = any(
                abs(target[j] - previous[j]) > 1e-9
                for j in target
                if j not in self.tool_channels
            )
            tool_changed = any(
                abs(target[j] - previous[j]) > 1e-9 for j in self.tool_channels
            )
            if tool_changed and not body_changed:
                break
            group.append(action)
            previous = target
            if "checkpoint" in action or action.get("stop", False):
                break
        return group

    def _run_motion_group(self, group):
        from trajectory import WaypointTrajectory

        actual = self.controller.read()
        targets, previous = [], self.targets.copy()
        for action in group:
            previous = {**previous, **action["target"]}
            targets.append(previous)
        # Retain an actively commanded grasp despite blocked-jaw feedback.
        for j in self.tool_channels:
            if all(abs(t[j] - self.targets[j]) < 1e-9 for t in targets):
                actual[j] = self.targets[j]
        limits = {
            j: (min(lo, actual[j]), max(hi, actual[j]))
            for j, (lo, hi) in getattr(self.controller, "limits", {}).items()
        }
        trajectory = WaypointTrajectory(
            actual,
            targets,
            speed=min(a.get("speed", self.speed) for a in group),
            limits=limits,
        )
        self.trajectory_validator(trajectory, group)
        start_index = self.index
        previous_time = time.time()

        def crossed(index, state):
            nonlocal previous_time
            action = group[index]
            self.publish(action, state)
            now = time.time()
            self.records.append(
                {
                    "name": action.get("name", str(self.index + 1)),
                    "start": previous_time,
                    "end": now,
                    "segments": 1,
                    "interpolation": "cubic_hermite",
                    "stopped": index == len(group) - 1,
                }
            )
            previous_time = now
            self.targets = targets[index]
            self.index += 1

        state = self.controller.move_trajectory(
            trajectory,
            on_waypoint=crossed,
            tool_channels=self.tool_channels,
            max_delta=self.max_delta,
        )
        if self.index != start_index + len(group):
            raise RuntimeError("Controller did not report the completed trajectory")
        return group[-1], state

    def _run_single(self):
        action = self.actions[self.index]
        target = {**self.targets, **action["target"]}
        before = time.time()
        segments = 0
        while True:
            actual = self.controller.read()
            distance = max(abs(target[j] - actual[j]) for j in target)
            # Recompute subdivisions from measured state to retain margin
            # when the device has a small static tracking error.
            final = distance <= self.max_delta
            fraction = 1.0 if final else self.max_delta * 0.9 / distance
            waypoint = {
                j: actual[j] + fraction * (v - actual[j]) for j, v in target.items()
            }
            for joint in self.tool_channels:
                if abs(target[joint] - actual[joint]) <= self.max_delta:
                    waypoint[joint] = target[joint]
            state = self.controller.move(
                waypoint,
                speed=action.get("speed", self.speed),
                max_delta=self.max_delta,
            )
            segments += 1
            self.publish(action, state)
            if final:
                break
            after = self.controller.read()
            if max(abs(target[j] - after[j]) for j in target) >= distance - 1e-6:
                raise RuntimeError("Subdivided action made no progress")
        self.targets = target
        self.index += 1
        self.records.append(
            {
                "name": action.get("name", str(self.index)),
                "start": before,
                "end": time.time(),
                "segments": segments,
            }
        )
        return action, state

    def run(self):
        if self.waiting or self.failed:
            raise RuntimeError(
                "Review the pending checkpoint or execution failure first"
            )
        if self.started_at is None:
            actual = self.controller.read()
            for joint, expected in (self.expected_start or {}).items():
                tolerance = self.start_tolerance.get(joint, 0.0)
                if not math.isfinite(tolerance) or tolerance < 0:
                    raise ValueError("Start tolerances must be nonnegative and finite")
                if abs(actual[joint] - expected) > tolerance:
                    raise ValueError(f"Plan start changed: {joint}")
            self.started_at = time.time()
        try:
            while self.index < len(self.actions):
                group = (
                    self._motion_group()
                    if self.trajectory_validator is not None
                    else []
                )
                if group:
                    action, state = self._run_motion_group(group)
                else:
                    action, state = self._run_single()
                if "checkpoint" in action:
                    self.waiting = True
                    checkpoint = {
                        "after_action": action.get("name", str(self.index)),
                        "arrived_at": time.time(),
                    }
                    self.checkpoints.append(checkpoint)
                    observation = self.observe(action)
                    checkpoint["observation_ready_at"] = time.time()
                    return {
                        "status": "checkpoint",
                        "next_action": self.index,
                        "checkpoint": action["checkpoint"],
                        "observation": observation,
                        "state": state,
                        "elapsed_s": time.time() - self.started_at,
                    }
            return {
                "status": "complete",
                "actions": self.index,
                "elapsed_s": time.time() - self.started_at,
                "records": self.records,
                "checkpoints": self.checkpoints,
            }
        except BaseException:
            self.failed = True
            self.controller.hold()
            raise
