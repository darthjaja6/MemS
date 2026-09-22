"""Shape-preserving cubic Hermite trajectories in controller-native units."""

import math
import numpy as np
from scipy.interpolate import CubicHermiteSpline, PchipInterpolator


class WaypointTrajectory:
    """Pass through joint waypoints; start/end at rest, retain interior tangents.

    PCHIP supplies monotone slopes, with zero endpoint velocities. Uniform time
    scaling enforces the exact polynomial speed extrema. Geometry validation
    belongs to the caller and must evaluate this curve, not just its knots.
    """

    def __init__(self, start, targets, *, speed, limits=None, max_acceleration=None):
        self.names = tuple(start)
        if not self.names or not targets or not math.isfinite(speed) or speed <= 0:
            raise ValueError("Provide a start, targets, and positive finite speed")
        points = [dict(start)]
        for target in targets:
            if not set(target) <= set(self.names):
                raise ValueError("Unknown trajectory channel")
            points.append({**points[-1], **target})
        self.points = np.array(
            [[p[n] for n in self.names] for p in points], dtype=float
        )
        if not np.isfinite(self.points).all():
            raise ValueError("Nonfinite trajectory position")
        for i, name in enumerate(self.names):
            if limits and name in limits:
                lo, hi = limits[name]
                if (
                    self.points[:, i].min() < lo - 1e-9
                    or self.points[:, i].max() > hi + 1e-9
                ):
                    raise ValueError(f"Trajectory outside joint limits: {name}")
        durations = np.maximum(
            0.5, np.max(np.abs(np.diff(self.points, axis=0)), axis=1) / speed
        )
        knots = np.r_[0.0, np.cumsum(durations)]
        slopes = PchipInterpolator(knots, self.points, axis=0).derivative()(knots)
        slopes[0] = slopes[-1] = 0.0
        curve = CubicHermiteSpline(knots, self.points, slopes, axis=0)
        peak_speed, peak_acceleration = self.extrema(curve)
        scale = max(1.0, float(peak_speed.max()) / speed)
        if max_acceleration is not None:
            if not math.isfinite(max_acceleration) or max_acceleration <= 0:
                raise ValueError("Acceleration limit must be positive and finite")
            scale = max(
                scale, math.sqrt(float(peak_acceleration.max()) / max_acceleration)
            )
        self.times = knots * scale
        self.curve = CubicHermiteSpline(self.times, self.points, slopes / scale, axis=0)
        self.duration = float(self.times[-1])
        self.peak_speed = peak_speed / scale
        self.peak_acceleration = peak_acceleration / scale**2
        self.speed = float(speed)

    @staticmethod
    def extrema(curve):
        """Exact |velocity| and |acceleration| maxima of cubic segments."""
        peak_v = np.zeros(curve.c.shape[2])
        peak_a = np.zeros_like(peak_v)
        for i, dt in enumerate(np.diff(curve.x)):
            a, b, c, _ = curve.c[:, i, :]
            peak_v = np.maximum(
                peak_v, np.maximum(np.abs(c), np.abs(3 * a * dt**2 + 2 * b * dt + c))
            )
            root = np.divide(-b, 3 * a, out=np.zeros_like(a), where=np.abs(a) > 1e-15)
            inside = (np.abs(a) > 1e-15) & (root > 0) & (root < dt)
            peak_v = np.maximum(
                peak_v, np.where(inside, np.abs(3 * a * root**2 + 2 * b * root + c), 0)
            )
            peak_a = np.maximum(
                peak_a, np.maximum(np.abs(2 * b), np.abs(6 * a * dt + 2 * b))
            )
        return peak_v, peak_a

    def at(self, seconds, derivative=0):
        values = self.curve(np.clip(seconds, 0.0, self.duration), nu=derivative)
        return {name: float(value) for name, value in zip(self.names, values)}

    def samples(self, max_step=0.05):
        if not math.isfinite(max_step) or max_step <= 0:
            raise ValueError("Sample interval must be positive and finite")
        times = np.unique(
            np.r_[
                np.linspace(0, self.duration, math.ceil(self.duration / max_step) + 1),
                self.times,
            ]
        )
        return [(float(t), self.at(t)) for t in times]

    def description(self):
        return {
            "method": "cubic_hermite_pchip_slopes",
            "duration": self.duration,
            "knot_times": self.times.tolist(),
            "channels": list(self.names),
            "positions": self.points.tolist(),
            "velocities": self.curve(self.times, 1).tolist(),
            "peak_speed": self.peak_speed.tolist(),
            "peak_acceleration": self.peak_acceleration.tolist(),
        }
