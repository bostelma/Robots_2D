from dataclasses import dataclass

import numpy as np
from spatialmath import SE3, Twist3

from robots_2d.classes.frame import Frame
from robots_2d.classes.joint import Joint, JointType
from robots_2d.classes.link import Link
from robots_2d.classes.robot import Robot


@dataclass
class Robot3R(Robot):
    """Planar three-link revolute robot.
    
    The robot consists of three revolute joints and three links arranged in an
    open serial chain. Joint axes are parallel to the z-axis and expressed in
    the space frame.

    Parameters
    ----------
    l1 : float, optional
        Length of the first link. Defaults to 1.0.
    l2 : float, optional
        Length of the second link. Defaults to 1.0.
    l3 : float, optional
        Length of the third link. Defaults to 1.0.

    Attributes
    ----------
    l1 : float
        Length of the first link.
    l2 : float
        Length of the second link.
    l3 : float
        Length of the third link.

    Notes
    -----
    The end-effector position is constrained to the x/y plane. Its orientation
    in the plane is given by
    
    phi = q1 + q2 + q3
    
    For a specified end-effector position and orientation, the inverse
    kinematics problem generally has two solutions. The solution whose first
    joint angle requires the smaller change from the current configuration is
    selected.
    
    The position-only reachable workspace is bounded by 
    
    max(0, 2 * max(l1, l2, l3) - (l1 + l2 + l3))
        <= sqrt(x**2 + y**2)
        <= l1 + l2 + l3
        
    When an end-effector orientation is specified, reachability additionally
    depends on the orientation because the position of the wrist point is
    constrained by the third link. 
    """

    l1: float = 1.0
    l2: float = 1.0
    l3: float = 1.0

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
        link3_frame = Frame(
            name = "link3",
            M = SE3.Trans(self.l1 + self.l2, 0, 0)
        )
        link_end_frame = Frame(
            name = "link_end",
            M = SE3.Trans(self.l1 + self.l2 + self.l3, 0, 0)
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

        link3 = Link(
            name = "link3",
            frame = link3_frame,
            length = self.l3
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
            q = 0.0,
        )

        joint2 = Joint(
            name = "joint2",
            parent = link1_frame,
            child = link2_frame,
            type = JointType.REVOLUTE,
            screw_axis = Twist3([0, -self.l1, 0, 0, 0, 1]),
            q = 0.0,
        )

        joint3 = Joint(
            name = "joint3",
            parent = link2_frame,
            child = link3_frame,
            type = JointType.REVOLUTE,
            screw_axis = Twist3([0, -(self.l1 + self.l2), 0, 0, 0, 1]),
            q = 0.0,
        )

        joint4 = Joint(
            name = "joint4",
            parent = link3_frame,
            child = link_end_frame,
            type = JointType.FIXED,
            screw_axis = Twist3([0, -(self.l1 + self.l2 + self.l3), 0, 0, 0, 1]),
            q = 0.0
        )

        self.frames = [
            world_frame, link1_frame, link2_frame, link3_frame, link_end_frame
        ]
        self.links = [
            link1, link2, link3, link_end
        ]
        self.joints = [
            joint1, joint2, joint3, joint4
        ]

    def inverse_kinematics(self, target):
        return np.array([0.0, 0.0, 0.0])