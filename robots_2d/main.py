from robots.robot_2r import Robot2R

from robots_2d.tools.visualization import visualize

if __name__ == "__main__":

    robot = Robot2R(
        l1 = 1.0,
        l2 = 0.75,
    )

    # Compute Inverse Kinematics
    robot.move_joints(
        *robot.inverse_kinematics(
            -1.0,                    # x-coordinate
            1.0                     # y-coordinate
        )
    )

    visualize(
        robot,
        me = True
    )