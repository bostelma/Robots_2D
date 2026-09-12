import matplotlib.pyplot as plt
import numpy as np

from robots_2d.classes.frame import Frame
from robots_2d.classes.joint import JointType
from robots_2d.classes.robot import Robot


def visualize(robot: Robot, poses: dict[str,Frame]):

    _, ax = plt.subplots()

    # Plot all links
    for link in robot.links:

        # Transformation matrix
        T = poses[link.name]

        # Change reference frame for start and end point
        p_start = T * np.array([0.0, 0.0, 0.0])
        p_end = T * np.array([link.length, 0.0, 0.0])

        # Draw the end-effector
        if link.name == "link_end":

            # Plot a circle
                circle = plt.Circle(
                    (p_start[0], p_start[1]),   # center coordinates
                    radius = 0.075,             # Radius 
                    color = 'green',            # Color
                    fill = True,                # Fill circle
                    zorder = 2                  # Draw in front
                )
                ax.add_patch(circle)

        # Draw a normal link
        else:

            # Plot a line
            ax.plot(
                [p_start[0], p_end[0]],     # xs
                [p_start[1], p_end[1]],     # ys
                linewidth = 5,              # Line width
                color = 'black',            # Color
                zorder = 1                  # Draw in the back
            )

    # Plot all joints
    for joint in robot.joints:

        # Revolute joints
        if joint.type == JointType.REVOLUTE:

            # Transformation matrix
            T = poses[joint.child.name]

            # Change reference frame for joint origin
            p_joint = T * np.array([0.0, 0.0, 0.0])

            # Plot a circle
            circle = plt.Circle(
                (p_joint[0], p_joint[1]),   # center coordinates
                radius = 0.075,             # Radius 
                color = 'red',              # Color
                fill = True,                # Fill circle
                zorder = 2                  # Draw in front
            )
            ax.add_patch(circle)

        elif joint.type == JointType.FIXED:

            # Don't draw fixed joints
            pass

        else:
            raise RuntimeError(
                f"Plotting of joint type {joint.type} no implemented yet!"    
            )

    # Adjust plotting parameters
    ax.axis("off")
    ax.set_aspect(
        'equal',
        adjustable='box'
    )

    plt.show()