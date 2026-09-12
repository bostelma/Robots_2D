from dataclasses import dataclass

import numpy as np
from spatialmath import SE3, Twist3

from robots_2d.classes.frame import Frame
from robots_2d.classes.joint import Joint, JointType
from robots_2d.classes.link import Link
from robots_2d.classes.robot import Robot


@dataclass
class Robot2R(Robot):

    l1: float = 1.0
    l2: float = 1.0
    q1: float = 0.0
    q2: float = 0.0

    def __post_init__(self):

        # Frames
        world_frame = Frame("world")
        link1_frame = Frame(
            name = "link1",
            M = SE3.Trans(0, 0, 0)
        )
        link2_frame = Frame(
            name = "link2",
            M = SE3.Trans(self.l1, 0, 0)
        )
        link_end_frame = Frame(
            name = "link_end",
            M = SE3.Trans(self.l1 + self.l2, 0, 0)
        )

        # Links
        link1 = Link(
            name = "link1",
            frame = link1_frame,
            length = self.l1,
        )

        link2 = Link(
            name = "link2",
            frame = link2_frame,
            length = self.l2,
        )

        link_end = Link(
            name = "link_end",
            frame = link_end_frame,
            length = 0.0
        )

        # Joints
        joint1 = Joint(
            name = "joint1",
            parent = world_frame,
            child = link1_frame,
            type = JointType.REVOLUTE,
            screw_axis = Twist3([0, 0, 0, 0, 0, 1]),
            q = self.q1,
        )


        # Joint 2
        joint2 = Joint(
            name = "joint2",
            parent = link1_frame,
            child = link2_frame,
            type = JointType.REVOLUTE,
            screw_axis = Twist3([0, -self.l1, 0, 0, 0, 1]),
            q = self.q2,
        )

        # Joint 3
        joint3 = Joint(
            name = "joint3",
            parent = link2_frame,
            child = link_end_frame,
            type = JointType.FIXED,
            screw_axis = Twist3([0, -(self.l1 + self.l2), 0, 0, 0, 1]),
            q = 0.0
        )

        self.frames = [world_frame, link1_frame, link2_frame, link_end_frame]
        self.links = [link1, link2, link_end]
        self.joints = [joint1, joint2, joint3]

    def inverse_kinematics(self, x, y):

        dist = np.sqrt(x*x + y*y)

        # No solution
        if dist < self.l2 - self.l1 or dist > self.l1 + self.l2:

            raise ValueError(
                "No inverse kinematics solution for requested tip position!"
            )

        # One solution on the inner workspace boundary
        elif dist == self.l1 - self.l2:

            q1 = np.atan2(y, x)
            q2 = np.pi

            return q1, q2

        # One solution on the outer workspace boundary
        elif dist == self.l1 + self.l2:

            q1 = np.atan2(y, x)
            q2 = 0

            return q1, q2

        # Two solutions
        else:

            gamma = np.atan2(y, x)
            alpha = np.acos((self.l1**2+dist**2-self.l2**2)/(2*self.l1*dist))
            betha = np.acos((self.l1**2+self.l2**2-dist**2)/(2*self.l1*self.l2))

            sol1 = gamma - alpha, np.pi - betha
            sol2 = gamma + alpha, betha - np.pi

            # Select the solution with smaller angle change of q1
            if np.abs(sol1[0] - self.q1) < np.abs(sol2[0] - self.q1):
                return sol1
            else:
                return sol2
        
