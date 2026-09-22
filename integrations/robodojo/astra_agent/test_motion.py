"""Gripper collision geometry regressions using real MPlib, without a simulator."""
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

import mplib
from mplib.collision_detection import fcl
import numpy as np

from .motion import MotionPlanner, JointPlanner, SIDES


class GripperPlanningTests(unittest.TestCase):
    def planner(self, obstacle=None):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        root = Path(temp.name)
        urdf = ET.Element('robot', name='slider')
        for name in ('base', 'tool', 'finger'):
            link = ET.SubElement(urdf, 'link', name=name)
            if name == 'finger':
                geometry = ET.SubElement(ET.SubElement(link, 'collision'), 'geometry')
                ET.SubElement(geometry, 'sphere', radius='.002')
        for name, parent, child, axis, limit in [('arm', 'base', 'tool', '1 0 0', '1'),
                                                ('jaw', 'tool', 'finger', '0 1 0', '.04')]:
            joint = ET.SubElement(urdf, 'joint', name=name, type='prismatic')
            ET.SubElement(joint, 'parent', link=parent)
            ET.SubElement(joint, 'child', link=child)
            ET.SubElement(joint, 'axis', xyz=axis)
            ET.SubElement(joint, 'limit', lower='0', upper=limit, velocity='5', effort='100')
        path = root / 'robot.urdf'
        ET.ElementTree(urdf).write(path)
        robots = {}
        for side, y in zip(SIDES, (0., 1.)):
            base = np.eye(4)
            base[1, 3] = y
            robots[side] = dict(urdf=str(path), tip='tool', joint_names=['arm'],
                                gripper_joints=['jaw'], gripper_scale=[-.01, .04],
                                base_transform=base, tool=dict(point_in_tip=[0, 0, 0]))
        obstacles = []
        if obstacle is not None:
            obstacles.append(fcl.FCLObject('obstacle', mplib.Pose(obstacle, [1, 0, 0, 0]),
                             [fcl.CollisionObject(fcl.Sphere(.002))], [mplib.Pose()]))
        planner = MotionPlanner.__new__(MotionPlanner)
        JointPlanner.__init__(planner, robots, root / 'planning', period_s=.04,
                              velocity=5., acceleration=10., obstacles=obstacles)
        planner.scene = dict(robots=robots)
        planner.attached = True
        return planner

    def state(self, opening=1.):
        return {f'{side}_{part}_joint_state': [opening if part == 'ee' else .1]
                for side in SIDES for part in ('arm', 'ee')}

    def waypoint(self, opening, x=None, steps=16):
        return dict(left_gripper=opening, right_gripper=1., steps=steps,
                    left_move=x is not None, right_move=False,
                    left_pose=[x, 0, 0, 1, 0, 0, 0])

    def test_requested_geometry_persists_into_later_arm_phase(self):
        # The open finger hits this obstacle; the closed finger passes below it.
        planner = self.planner([.2, .04, 0])
        actions = planner.compile(self.state(), [self.waypoint(0., steps=2),
                                                self.waypoint(0., x=.3)])
        self.assertEqual(len(actions), 18)
        np.testing.assert_allclose(actions[0]['left_arm_joint_state'], [.1])
        np.testing.assert_allclose(actions[-1]['left_arm_joint_state'], [.3], atol=1.01e-3)
        np.testing.assert_allclose(planner.robot.get_qpos()[2:], [0., .04])
        self.assertTrue(all(action['left_ee_joint_state'][0] == 0 for action in actions))
        self.assertTrue(all(action['right_ee_joint_state'][0] == 1 for action in actions))
        with self.assertRaisesRegex(ValueError, 'limit is 17'):
            planner.compile(self.state(), [self.waypoint(0., steps=2),
                                           self.waypoint(0., x=.3)], max_steps=17)

    def test_transition_rejects_collision_between_clear_endpoints(self):
        planner = self.planner([.1, .02, 0])
        with self.assertRaisesRegex(ValueError, 'collides'):
            planner.compile(self.state(), [self.waypoint(0., steps=2)])

    def test_moving_phase_checks_opening_range_along_path(self):
        planner = self.planner([.2, .04, 0])
        with self.assertRaisesRegex(ValueError, 'collides'):
            planner.compile(self.state(), [self.waypoint(0., x=.3)])

    def test_new_sequence_starts_from_measured_opening(self):
        planner = self.planner([.1, .04, 0])
        with self.assertRaisesRegex(ValueError, 'collides'):
            planner.compile(self.state(), [self.waypoint(0., steps=2)])
        # A fresh measured closed state is valid, even after a failed compile.
        actions = planner.compile(self.state(opening=0.), [self.waypoint(0., steps=2)])
        self.assertEqual(len(actions), 2)


if __name__ == '__main__':
    unittest.main()
