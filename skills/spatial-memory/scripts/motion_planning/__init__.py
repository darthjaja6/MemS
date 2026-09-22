"""Reusable joint planning: Drake goals, OMPL routes, PCHIP/TOPPRA timing.

JointPlanner receives named robot models, FCL obstacles and controller limits.
set_state takes all joints in planner.names order: active joints, then passive
joints. route takes {robot_name: ToolGoal}; omitted robots hold measured joints.
timed_path returns joint samples including the start at the configured period.
Requires Drake, MPlib, SciPy, trimesh and TOPPRA; no simulator or model inference.
"""
import mplib
from mplib.planning.ompl import OMPLPlanner, FixedJoint
import numpy as np
from scipy.interpolate import PchipInterpolator
import toppra as ta
import toppra.algorithm as algo

from .ik import CartesianIK, ToolGoal
from .model import prepare_model

__all__ = ['JointPlanner', 'ToolGoal']


class JointPlanner:
    def __init__(self, robots, directory, *, period_s, velocity, acceleration, obstacles=()):
        urdf, srdf, self.names, self.slices = prepare_model(robots, directory)
        self.dof = sum(len(r['joint_names']) for r in robots.values())
        self.period = float(period_s)
        self.velocity = np.broadcast_to(velocity, (self.dof,)).copy()
        self.acceleration = np.broadcast_to(acceleration, (self.dof,)).copy()
        if (not np.isfinite(self.period) or self.period <= 0 or
                not np.isfinite([self.velocity, self.acceleration]).all() or
                np.any(self.velocity <= 0) or np.any(self.acceleration <= 0)):
            raise ValueError('Controller period and joint speed/acceleration limits must be positive')
        self.robot = mplib.ArticulatedModel(str(urdf), str(srdf), joint_names=self.names)
        self.robot.set_move_group([name + '_' + r['tip'] for name, r in robots.items()])
        if self.robot.get_move_group_joint_indices() != list(range(self.dof)):
            raise ValueError('joint_names must cover the active chains to the configured tips')
        self.limits = np.concatenate(self.robot.get_pinocchio_model().get_joint_limits())
        self.bounds = self.limits[:self.dof]
        self.world = mplib.PlanningWorld([self.robot], list(obstacles))
        self.ompl = OMPLPlanner(self.world)
        self.ik = CartesianIK(urdf, srdf, self.names, robots, self.slices)

    def set_state(self, joints):
        joints = np.asarray(joints, dtype=float)
        if joints.shape != (len(self.names),) or not np.isfinite(joints).all():
            raise ValueError('Supply a finite measured value for every configured joint')
        if np.any(joints < self.limits[:, 0] - 1e-7) or np.any(joints > self.limits[:, 1] + 1e-7):
            raise ValueError('Measured state exceeds robot limits')
        self.robot.set_qpos(joints, True)
        self.check(joints[:self.dof])

    def check(self, q):
        q = np.asarray(q)
        if q.shape != (self.dof,) or not np.isfinite(q).all():
            raise ValueError('Joint path has invalid dimensions or non-finite values')
        if np.any(q < self.bounds[:, 0] - 1e-7) or np.any(q > self.bounds[:, 1] + 1e-7):
            raise ValueError('Joint path exceeds robot limits')
        self.world.set_qpos_all(q)
        collisions = self.world.check_collision()
        if collisions:
            pairs = ', '.join(f'{c.link_name1}:{c.link_name2}' for c in collisions)
            raise ValueError(f'Joint path collides: {pairs}')

    def route(self, goals):
        current = self.robot.get_qpos().copy()
        start = current[:self.dof]
        if not goals:
            return start[None, :]
        if not goals.keys() <= self.slices.keys():
            raise ValueError('Goal names must identify configured robots')
        try:
            target = self.ik.solve(current, goals)[:self.dof]
            self.world.set_qpos_all(target)
            if self.world.check_collision():
                target = self.ik.solve(current, goals, avoid_collision=True)[:self.dof]
            self.check(target)
            self.world.set_qpos_all(start)
            if np.max(np.abs(target - start)) < 1e-8:
                return start[None, :]
            fixed = {FixedJoint(0, i, start[i]) for name, span in self.slices.items() if name not in goals
                     for i in range(span.start, span.stop)}
            status, route = self.ompl.plan(start, [target], time=2., range=.1,
                                           fixed_joints=fixed, simplify=True)
            if status != 'Exact solution':
                raise ValueError(f'OMPL planning failed: {status}')
            if not np.allclose(route[0], start, atol=1e-8, rtol=0):
                raise ValueError('OMPL changed the measured start')
            self.check(route[-1])
            return route
        except Exception:
            self.robot.set_qpos(current, True)
            raise

    def timed_path(self, route, minimum_steps):
        """Smooth and time a route, then collision-check samples at 4x control rate."""
        route = np.asarray(route, dtype=float)
        # Shared segment endpoints and stationary waypoints can repeat.
        route = route[np.r_[True, np.linalg.norm(np.diff(route, axis=0), axis=1) > 1e-10]]
        if len(route) == 1:
            self.check(route[0])
            self.last_timing = dict(max_joint_speed=0., max_joint_acceleration=0.)
            return np.repeat(route, minimum_steps + 1, axis=0)
        progress = np.r_[0., np.cumsum(np.linalg.norm(np.diff(route, axis=0), axis=1))]
        spline = PchipInterpolator(progress, route)
        path = ta.SimplePath(progress, route, spline.derivative()(progress))
        constraints = [ta.constraint.JointVelocityConstraint(self.velocity),
                       ta.constraint.JointAccelerationConstraint(self.acceleration)]
        trajectory = algo.TOPPRA(constraints, path, parametrizer='ParametrizeConstAccel').compute_trajectory(0., 0.)
        if trajectory is None:
            raise ValueError('TOPPRA could not time the path')
        steps = max(minimum_steps, int(np.ceil(trajectory.duration / self.period)))
        times = np.linspace(0., trajectory.duration, steps + 1)
        dense = np.linspace(0., trajectory.duration, steps * 4 + 1)
        scale = trajectory.duration / (steps * self.period)
        self.last_timing = dict(max_joint_speed=float(np.abs(trajectory(dense, 1)).max()) * scale,
                               max_joint_acceleration=float(np.abs(trajectory(dense, 2)).max()) * scale**2)
        for q in trajectory(dense):
            self.check(q)
        return trajectory(times)
