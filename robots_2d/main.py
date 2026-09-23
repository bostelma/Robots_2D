import numpy as np
from robots.robot_2r import Robot2R

from robots_2d.tools.visualization import animate_robot

if __name__ == "__main__":

    robot = Robot2R(
        l1 = 1.0,
        l2 = 0.75,
    )

    # Display the degree of freedom of the robot
    if True:
        dof = robot.dof()
        unit = "degree" if dof == 1 else "degrees"
        print(f"The robot has {dof} {unit} of freedom.")

    # Inverse Kinematics using analytical solution
    if True:
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
    if True:
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

    # Trajectories: P2P in joint space
    if True:
        q_start = np.array([0.0, 0.0])
        q_end = np.array([2 * np.pi, - 8 * np.pi])

        qs = robot.trajectory_p2p_joint_space(
            q_start = q_start,
            q_end = q_end,
            T = 5,
            f = 50,
            time_scaling = 'poly3'      # Select from: {linear, poly3, poly5}
        )

        animate_robot(
            robot,
            qs,
            me = True,
            interval = 1000 / 50
        )

    # Trajectories: P2P in joint space
    if True:
        X_start = np.array([1.25, 0.0])
        X_end = np.array([-1.25, 1.0])

        qs = robot.trajectory_p2p_cartesisan_space(
            X_start = X_start,
            X_end = X_end,
            T = 1,
            f = 50,
            time_scaling = 'poly5'      # Select from: {linear, poly3, poly5}
        )

        animate_robot(
            robot,
            qs,
            me = True,
            interval = 1000 / 50
        )

    # Trajectoreis: Via points in joint space
    if True:

        N = 10

        via_points = np.random.rand(N, 2) * 2 * np.pi
        Ts = np.linspace(0.0, via_points.shape[0], via_points.shape[0])

        qs = robot.trajectory_via_points_joint_space(
            via_points = via_points,
            Ts = Ts,
            f = 50
        )

        animate_robot(
            robot,
            qs,
            me = True,
            interval = 1000 / 50
        )

    # Trajectoreis: Via points in cartesian space
    if True:

        N = 10

        max_arm_length = robot.links[0].length + robot.links[1].length
        min_arm_length = max(0.0, robot.links[0].length - robot.links[1].length)

        theta = np.random.uniform(0, 2 * np.pi, N)
        radius = np.sqrt(
            np.random.uniform(min_arm_length**2, max_arm_length**2, N)
        )

        x = radius * np.cos(theta)
        y = radius * np.sin(theta)

        via_points = np.column_stack((x, y))
        Ts = np.linspace(0.0, via_points.shape[0], via_points.shape[0])

        qs = robot.trajectory_via_points_cartesian_space(
            via_points = via_points,
            Ts = Ts,
            f = 50
        )

        animate_robot(
            robot,
            qs,
            me = True,
            interval = 1000 / 50
        )