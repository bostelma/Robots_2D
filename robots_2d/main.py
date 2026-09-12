import numpy as np
from robots.robot_2r import Robot2R

from robots_2d.tools.visualization import visualize

if __name__ == "__main__":

    robot = Robot2R(
        l1 = 1.0,
        l2 = 1.0,
        q1 = np.pi / 4,
        q2 = np.pi / 8,
    )

    poses = robot.forward_kinematics()

    visualize(
        robot,
        me = True
    )