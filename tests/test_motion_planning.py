"""Exercise the shared planner on scalar joints, arbitrary names and unequal arms."""
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'skills/spatial-memory/scripts'))
try:
    import mplib
    from mplib.collision_detection import fcl
    from motion_planning import JointPlanner, ToolGoal
except ModuleNotFoundError:
    JointPlanner = None


@unittest.skipIf(JointPlanner is None, 'Install the optional motion planning dependencies')
class MotionPlanningTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def robot(self, name, yaw_joint=True, base=None):
        root = ET.Element('robot', name=name)
        ET.SubElement(root, 'link', name='mounting_plate')
        parent, names = 'mounting_plate', []
        axes = [('x', 'prismatic', '1 0 0'), ('y', 'prismatic', '0 1 0'), ('z', 'prismatic', '0 0 1')]
        if yaw_joint:
            axes.append(('yaw', 'revolute', '0 0 1'))
        for label, kind, axis in axes:
            child = label + '_link'
            ET.SubElement(root, 'link', name=child)
            joint = ET.SubElement(root, 'joint', name=label, type=kind)
            ET.SubElement(joint, 'parent', link=parent)
            ET.SubElement(joint, 'child', link=child)
            ET.SubElement(joint, 'axis', xyz=axis)
            ET.SubElement(joint, 'limit', lower='-3.14', upper='3.14', velocity='2', effort='100')
            names.append(label)
            parent = child
        geometry = ET.SubElement(ET.SubElement(root.findall('link')[-1], 'collision'), 'geometry')
        ET.SubElement(geometry, 'sphere', radius='.02')
        # A passive joint outside the tool chain must remain at the measured value.
        ET.SubElement(root, 'link', name='finger')
        finger = ET.SubElement(root, 'joint', name='jaw', type='prismatic')
        ET.SubElement(finger, 'parent', link=parent)
        ET.SubElement(finger, 'child', link='finger')
        ET.SubElement(finger, 'axis', xyz='0 1 0')
        ET.SubElement(finger, 'limit', lower='0', upper='.04', velocity='1', effort='10')
        path = self.root / (name + '.urdf')
        ET.ElementTree(root).write(path)
        return dict(urdf=str(path), tip=parent, joint_names=names,
                    base_transform=np.eye(4) if base is None else base,
                    tool=dict(point_in_tip=[.03, 0, 0]))

    def planner(self, robots, **kwargs):
        return JointPlanner(robots, self.root / 'cache', period_s=.02,
                            velocity=1., acceleration=2., **kwargs)

    def tip(self, planner, name, robot):
        pin = planner.robot.get_pinocchio_model()
        pin.compute_forward_kinematics(planner.robot.get_qpos())
        pose = pin.get_link_pose(pin.get_link_names().index(name + '_' + robot['tip']))
        rotation = Rotation.from_quat(pose.q, scalar_first=True)
        return pose.p + rotation.apply(robot['tool']['point_in_tip']), rotation

    def test_single_arm_position_and_timing_start_at_measured_state(self):
        robot = self.robot('gantry')
        planner = self.planner({'gantry': robot})
        q0 = np.array([.1, -.1, .3, 3.13, .017])
        planner.set_state(q0)
        goal = ToolGoal([.3, .2, .4], Rotation.from_euler('z', -3.13).as_quat(scalar_first=True))
        route = planner.route({'gantry': goal})
        q = planner.timed_path(route, 10)
        np.testing.assert_array_equal(q[0], q0[:4])
        self.assertLess(np.max(np.abs(q[:, 3] - q0[3])), .05)
        self.assertLess(planner.last_timing['max_joint_speed'], 1.001)
        self.assertLess(planner.last_timing['max_joint_acceleration'], 2.001)
        point, _ = self.tip(planner, 'gantry', robot)
        np.testing.assert_allclose(point, goal.position, atol=goal.position_tolerance_m + 1e-6)
        self.assertAlmostEqual(planner.robot.get_qpos()[-1], q0[-1])

    def test_unequal_arms_and_base_transform_hold_omitted_arm(self):
        base = np.eye(4)
        base[:3, :3] = Rotation.from_euler('z', .7).as_matrix()
        base[0, 3] = 2.
        robots = {'picker': self.robot('picker'), 'slider': self.robot('slider', False, base)}
        planner = self.planner(robots)
        start = np.array([0, 0, .5, 0, .1, .2, .3, .015, .025])
        planner.set_state(start)
        point, rotation = self.tip(planner, 'slider', robots['slider'])
        goal = ToolGoal(point + [.1, -.1, .1], rotation.as_quat(scalar_first=True), fixed_orientation=True)
        q = planner.timed_path(planner.route({'slider': goal}), 10)
        np.testing.assert_allclose(q[:, :4], np.broadcast_to(start[:4], q[:, :4].shape), atol=1e-7)
        np.testing.assert_allclose(self.tip(planner, 'slider', robots['slider'])[0], goal.position, atol=goal.position_tolerance_m + 1e-6)
        np.testing.assert_allclose(planner.robot.get_qpos()[-2:], start[-2:])

    def test_explicit_pose_and_axis_constraints(self):
        robot = self.robot('tool')
        planner = self.planner({'tool': robot})
        requested = Rotation.from_euler('z', .4)
        for fixed in (True, False):
            planner.set_state([0, 0, .3, 0, .02])
            goal = ToolGoal([.1, .2, .4], requested.as_quat(scalar_first=True),
                            fixed_orientation=fixed,
                            axes=() if fixed else (([1, 0, 0], requested.apply([1, 0, 0])),))
            planner.route({'tool': goal})
            point, rotation = self.tip(planner, 'tool', robot)
            np.testing.assert_allclose(point, goal.position, atol=goal.position_tolerance_m + 1e-6)
            self.assertLess((rotation.inv() * requested).magnitude(), goal.angle_tolerance_rad + 1e-5)

    def test_collision_rejection_restores_start(self):
        robot = self.robot('tool')
        obstacle = fcl.FCLObject('obstacle', mplib.Pose([.5, 0, .3], [1, 0, 0, 0]),
                                [fcl.CollisionObject(fcl.Box(.1, .1, .1))], [mplib.Pose()])
        planner = self.planner({'tool': robot}, obstacles=[obstacle])
        start = [0, 0, .3, 0, .02]
        planner.set_state(start)
        with self.assertRaisesRegex(ValueError, 'collides'):
            planner.route({'tool': ToolGoal([.53, 0, .3], [1, 0, 0, 0], fixed_orientation=True)})
        np.testing.assert_array_equal(planner.robot.get_qpos(), start)


if __name__ == '__main__':
    unittest.main()
