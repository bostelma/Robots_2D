import unittest

from robots_2d.robots.robot_2r import Robot2R


class TestRobot2R(unittest.TestCase):

    # --------------------------------------------------------------------------
    # General properties
    # --------------------------------------------------------------------------
    

    def test_dof(self):

        robot = Robot2R()

        self.assertEqual(
            robot.dof(),
            2
        )

if __name__ == "__main__":
    unittest.main()