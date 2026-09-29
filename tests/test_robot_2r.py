import unittest

import numpy as np

from robots_2d.robots.robot_2r import Robot2R


class TestRobot2R(unittest.TestCase):

    def setUp(self):

        self.l1 = 1.0
        self.l2 = 0.5
    
        self.robot = Robot2R(
            l1 = self.l1,
            l2 = self.l2
        )

    # --------------------------------------------------------------------------
    # General properties
    # --------------------------------------------------------------------------
    
    def test_dof(self):

        self.assertEqual(
            self.robot.dof(),
            2
        )

    # --------------------------------------------------------------------------
    # Forward kinematics
    # --------------------------------------------------------------------------

    def test_zero_configuration(self):

        self.robot.move_joints(
            np.array([0.0, 0.0])
        )
        T = self.robot.forward_kinematics()["link_end"]

        np.testing.assert_allclose(
            T.A[:2, 3],
            [self.l1 + self.l2, 0.0],
            atol=1e-12,
        )

    def test_first_joint_90_degrees(self):

        self.robot.move_joints(
            np.array([np.pi / 2, 0.0])
        )
        T = self.robot.forward_kinematics()["link_end"]

        np.testing.assert_allclose(
            T.A[:2, 3],
            [0.0, self.l1 + self.l2],
            atol=1e-12,
        )

    def test_second_joint_90_degrees(self):

        self.robot.move_joints(
            np.array([0.0, np.pi / 2])
        )
        T = self.robot.forward_kinematics()["link_end"]

        np.testing.assert_allclose(
            T.A[:2, 3],
            [self.l1, self.l2],
            atol=1e-12,
        )

if __name__ == "__main__":
    unittest.main()