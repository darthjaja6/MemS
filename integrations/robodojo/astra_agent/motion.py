"""RoboDojo's X5 state/action encoding around the shared joint planner."""
from itertools import groupby
from pathlib import Path
import sys

import mplib
from mplib.collision_detection import fcl
import numpy as np
from scipy.spatial.transform import Rotation
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'skills/spatial-memory/scripts'))
from motion_planning import JointPlanner, ToolGoal
from assets import object_mesh
from kinematics import Robot

SIDES = ('left', 'right')
X5_MOTION = dict(period_s=.04, velocity=5., acceleration=10., support_extent_m=4.,
                 gripper_check_resolution_m=.001)


class MotionPlanner(JointPlanner):
    def __init__(self, scene, velocity=X5_MOTION['velocity'], acceleration=X5_MOTION['acceleration'],
                 *, root='.', contacts=()):
        self.scene = scene
        robots = {side: dict(scene['robots'][side]) for side in SIDES}
        for robot in robots.values():
            config = yaml.safe_load(Path(robot['urdf']).with_name('curobo.yml').read_text())
            ignores = config['robot_cfg']['kinematics']['self_collision_ignore']
            robot['collision_exclusions'] = [(a, b) for a, others in ignores.items() for b in others]
        surfaces = []
        for name, surface in scene.get('surfaces', {}).items():
            if surface.get('object') in scene.get('objects', {}):
                continue
            normal = np.asarray(surface['normal'], dtype=float)
            length = np.linalg.norm(normal)
            normal /= length
            rotation = Rotation.align_vectors([normal], [[0., 0., 1.]])[0]
            extent = X5_MOTION['support_extent_m']  # Covers the X5 reachable workspace.
            pose = mplib.Pose(normal * (-surface['offset_m'] / length - extent / 2),
                              rotation.as_quat(scalar_first=True))
            surfaces.append(fcl.FCLObject(name, pose, [fcl.CollisionObject(fcl.Box(*([extent] * 3)))],
                                          [mplib.Pose()]))
        self.object_names = []
        for name, obj in scene.get('objects', {}).items():
            if obj.get('role') == 'marker':
                continue
            mesh = object_mesh(scene, name, root, world=False)
            geometry = fcl.BVHModel()
            geometry.begin_model(len(mesh.faces), len(mesh.vertices))
            geometry.add_sub_model(mesh.vertices, mesh.faces.astype(np.int32))
            geometry.end_model()
            surfaces.append(fcl.FCLObject(name, mplib.Pose(obj['position'], obj.get('quaternion', [1,0,0,0])),
                                          [fcl.CollisionObject(geometry)], [mplib.Pose()]))
            self.object_names.append(name)
        directory = Path(robots['left']['urdf']).parent / 'planning'
        super().__init__(robots, directory, period_s=X5_MOTION['period_s'],
                         velocity=velocity, acceleration=acceleration, obstacles=surfaces)
        self.tool_links = {}
        for side, robot in robots.items():
            model = Robot(robot['urdf'])
            links = [robot['tip']] + [model.joints[j]['child'] for j in robot['gripper_joints']]
            self.tool_links[side] = [side + '_' + link for link in links]
        acm = self.world.get_allowed_collision_matrix()
        known = set(self.robot.get_user_link_names()) | set(self.object_names) | set(scene.get('surfaces', {}))
        for first, second in contacts:
            first = scene.get('surfaces', {}).get(first, {}).get('object', first)
            second = scene.get('surfaces', {}).get(second, {}).get('object', second)
            left = self.tool_links.get(first, [first])
            right = self.tool_links.get(second, [second])
            if not set(left + right) <= known:
                raise ValueError('Contact pairs must name scene objects, surfaces, robot tools or links')
            acm.set_entry(left, right, True)
        self.attached = False
        self._gripper_sweep = None

    def gripper_joints(self, openings):
        joints = {}
        for side, value in zip(SIDES, openings):
            robot = self.scene['robots'][side]
            lo, hi = robot['gripper_scale']
            opening = lo + (hi - lo) * value
            # X5's actuator closes past URDF zero; clamp only the geometry representation.
            for name in robot['gripper_joints']:
                name = side + '_' + name
                joints[name] = np.clip(opening, *self.limits[self.names.index(name)])
        return joints

    def set_state(self, state):
        self._gripper_sweep = None
        joints = self.gripper_joints([state.get(s + '_ee_joint_state', [1])[0] for s in SIDES])
        for side in SIDES:
            robot = self.scene['robots'][side]
            joints.update(zip((side + '_' + n for n in robot['joint_names']), state[side + '_arm_joint_state']))
        q = [joints[name] for name in self.names]
        # Attach in measured configuration before checking the initial state.
        if not self.attached:
            self.robot.set_qpos(q, True)
            links = self.robot.get_user_link_names()
            for name in self.object_names:
                attachment = self.scene['objects'][name].get('attachment')
                if attachment:
                    side = attachment['robot']
                    tip = side + '_' + self.scene['robots'][side]['tip']
                    self.world.attach_object(name, self.robot.get_name(), links.index(tip), self.tool_links[side])
            self.attached = True
        super().set_state(q)

    def check(self, q):
        super().check(q)
        if self._gripper_sweep is None:
            return
        target = self.robot.get_qpos().copy()
        try:
            # Commands change the fingers and arms together. Check the sampled
            # opening range along the arm path without assuming instant closure.
            for passive in self._gripper_sweep[:-1]:
                full = target.copy()
                full[self.dof:] = passive
                self.robot.set_qpos(full, True)
                super().check(q)
        finally:
            self.robot.set_qpos(target, True)

    def route(self, waypoint):
        goals = {}
        for side in SIDES:
            if waypoint.get(side + '_move', True):
                pose = waypoint[side + '_pose']
                rotation = Rotation.from_quat(pose[3:], scalar_first=True)
                point = np.asarray(pose[:3]) + rotation.apply(self.scene['robots'][side]['tool']['point_in_tip'])
                axes = waypoint.get(side + '_axes')
                goals[side] = ToolGoal(point, pose[3:], axes=axes or (), fixed_orientation=axes is None)
        return super().route(goals)

    def compile(self, state, waypoints, max_steps=96):
        self.set_state(state)
        actions = []

        def phase(w):
            return tuple(float(w[s + '_gripper']) for s in SIDES), not any(w.get(s + '_move', True) for s in SIDES)

        for (grippers, _), group in groupby(waypoints, key=phase):
            if any(not 0 <= g <= 1 for g in grippers):
                raise ValueError('Gripper opening must be 0..1')
            start = self.robot.get_qpos().copy()
            target = start.copy()
            for name, opening in self.gripper_joints(grippers).items():
                target[self.names.index(name)] = opening
            travel = np.max(np.abs(target[self.dof:] - start[self.dof:]), initial=0.)
            samples = int(np.ceil(travel / X5_MOTION['gripper_check_resolution_m'])) + 1
            self._gripper_sweep = np.linspace(start[self.dof:], target[self.dof:], samples)
            self.robot.set_qpos(target, True)
            self.check(start[:self.dof])
            route = [start[:self.dof]]
            duration = 0
            for waypoint in group:
                count = int(waypoint['steps'])
                if not 1 <= count <= 64:
                    raise ValueError('Waypoint duration must be 1..64 control steps')
                duration += count
                route.extend(self.route(waypoint)[1:])
            positions = self.timed_path(np.asarray(route), duration)
            self._gripper_sweep = None
            for q in positions[1:]:
                action = {}
                for side, gripper in zip(SIDES, grippers):
                    action[side + '_arm_joint_state'] = q[self.slices[side]].astype(np.float32)
                    action[side + '_ee_joint_state'] = np.array([gripper], dtype=np.float32)
                actions.append(action)
            if len(actions) > max_steps:
                raise ValueError(f'Planned sequence needs {len(actions)} steps; limit is {max_steps}')
        if not actions:
            raise ValueError('A decision must produce at least one action')
        return actions
