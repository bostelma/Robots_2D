import numpy as np
from robots.robot_2r import Robot2R

from robots_2d.tools.visualization import animate_robot

if __name__ == "__main__":

    robot = Robot2R(
        l1 = 1.0,
        l2 = 0.75,
    )

    # Inverse Kinematics using analytical solution
    if False:
        qs = np.array(
            robot.inverse_kinematics(
                1.0,                    # x-coordinate
                1.0                     # y-coordinate
            )
        )[np.newaxis, :]

        animate_robot(
            robot,
            qs,
            me = True
        )

    # Inverse Kinematics using numerical solution
    if False:
        qs = np.array(
            robot.inverse_kinematics_num(
                1.0,                    # x-coordinate
                1.0                     # y-coordinate
            )
        )[np.newaxis, :]

        animate_robot(
            robot,
            qs,
            me = True
        )

    # Animate robot movement
    if True:
        q1s = np.linspace(0.0, 2 * np.pi, 250)
        q2s = np.linspace(0.2, -4 * np.pi, 250)

        qs = np.column_stack((q1s, q2s))

        animate_robot(
            robot,
            qs,
            me = True,
            interval = 50
        )