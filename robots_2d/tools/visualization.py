import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle, Ellipse

from robots_2d.classes.joint import JointType
from robots_2d.classes.robot import Robot


def animate_robot(robot: Robot, qs: np.ndarray, me: bool = False, interval: int = 50):

    # ------------------------------------------------------------
    # Create the figure
    # ------------------------------------------------------------

    _, ax = plt.subplots()

    # Set the visible area based on the length of the robot arm
    total_length = 0
    for link in robot.links:
        total_length += link.length

    ax.set_xlim(-total_length * 1.5, total_length * 1.5)
    ax.set_ylim(-total_length * 1.5, total_length * 1.5)

    ax.axis("off")
    ax.set_aspect("equal", adjustable="box")

    # ------------------------------------------------------------
    # Create artists once
    # ------------------------------------------------------------

    # Link lines
    link_lines = {}

    for link in robot.links:

        if link.name == "link_end":
            continue

        line, = ax.plot(
            [],
            [],
            linewidth = 5,
            color = "black",
            zorder = 1,
        )

        link_lines[link.name] = line

    # Joint circles
    joint_circles = {}

    for joint in robot.joints:

        if joint.type == JointType.REVOLUTE:

            circle = Circle(
                (0, 0),
                radius = 0.075,
                color = "red",
                fill = True,
                zorder = 2,
            )

            ax.add_patch(circle)
            joint_circles[joint.child.name] = circle

        elif joint.type == JointType.FIXED:
            pass

        else:
            raise RuntimeError(
                f"Plotting of joint type {joint.type} not implemented yet!"
            )

    # End-effector circle
    end_effector = Circle(
        (0, 0),
        radius = 0.075,
        color = "green",
        fill = True,
        zorder = 2,
    )

    ax.add_patch(end_effector)

    # Manipulability ellipse
    if me:
        ellipse = Ellipse(
            (0, 0),
            width = 0,
            height = 0,
            angle = 0,
            color = "green",
            fill = False,
            zorder = 2,
        )

        ax.add_patch(ellipse)

    # ------------------------------------------------------------
    # Update function
    # ------------------------------------------------------------

    def update(frame):

        q = qs[frame]

        # Set robot configuration
        robot.move_joints(*q)
        poses = robot.forward_kinematics()

        # Links
        for link in robot.links:

            if link.name == "link_end":
                continue

            T = poses[link.name]

            p_start = T * np.array([0.0, 0.0, 0.0])
            p_end = T * np.array([link.length, 0.0, 0.0])

            link_lines[link.name].set_data(
                [p_start[0], p_end[0]],
                [p_start[1], p_end[1]],
            )

        # Joints
        for joint in robot.joints:

            if joint.type != JointType.REVOLUTE:
                continue

            T = poses[joint.child.name]

            p_joint = T * np.array([0.0, 0.0, 0.0])

            joint_circles[joint.child.name].center = (
                p_joint[0],
                p_joint[1],
            )

        # End effector
        T = poses["link_end"]

        p_end_effector = T * np.array([0.0, 0.0, 0.0])

        end_effector.center = (
            p_end_effector[0],
            p_end_effector[1],
        )

        # Manipulability ellipse
        if me:

            center, width, height, angle = (
                robot.manipulability_ellipse()
            )

            ellipse.center = center
            ellipse.width = width
            ellipse.height = height
            ellipse.angle = angle

        return (
            list(link_lines.values())
            + list(joint_circles.values())
            + [end_effector]
            + ([ellipse] if me else [])
        )

    # ------------------------------------------------------------
    # Animation
    # ------------------------------------------------------------

    animation = FuncAnimation(
        ax.figure,
        update,
        frames = len(qs),
        interval = interval,
        blit = True,
        repeat = True,
    )

    plt.show()

    return animation