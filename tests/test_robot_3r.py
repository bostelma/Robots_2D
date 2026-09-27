import unittest

from robots_2d.robots.robot_3r import Robot3R


class TestRobot2R(unittest.TestCase):

    # --------------------------------------------------------------------------
    # General properties
    # --------------------------------------------------------------------------
    

    def test_dof(self):

        robot = Robot3R()

        self.assertEqual(
            robot.dof(),
            3
        )

if __name__ == "__main__":
    unittest.main()