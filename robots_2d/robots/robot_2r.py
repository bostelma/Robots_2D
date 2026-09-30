from dataclasses import dataclass

import numpy as np
from spatialmath import SE3, Twist3

from robots_2d.classes.frame import Frame
from robots_2d.classes.joint import Joint, JointType
from robots_2d.classes.link import Link
from robots_2d.classes.robot import Robot


@dataclass
class Robot2R(Robot):
    """Planar two-link revolute robot.
    
    The robot consists of two revolute joints and two links arranged in an open
    serial chain. Joint axes are parallel to the z-axis and expressed in the
    space frame.
    
    Parameters
    ----------
    l1 : float, optional
        Length of the first link. Defaults to 1.0.
    l2 : float, optional
        Length of the second link. Defaults to 1.0.
    
    Attributes
    ----------
    l1 : float
        Length of the first link.
    l2 : float
        Length of the second link.
        
    Notes
    -----
    The end-effector position is constrained to the x/y plane. For a reachable
    target position, the inverse kinematics problem generally has two solutions.
    The solution whose first joint angle requires the smaller change from the
    current q1 is selected. On the inner and outer workspace boundaries, the two
    solutions coincide. The reachable workspace is bounded by the distances 
    abs(l1 - l2) <= sqrt(x**2 + y**2) <= l1 + l2. 
    """

    l1: float = 1.0
    l2: float = 1.0

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
            child = link_end_frame,
            type = JointType.FIXED,
            screw_axis = Twist3([0, -(self.l1 + self.l2), 0, 0, 0, 1]),
            q = 0.0
        )

        self.frames = [world_frame, link1_frame, link2_frame, link_end_frame]
        self.links = [link1, link2, link_end]
        self.joints = [joint1, joint2, joint3]

    def inverse_kinematics(self, target, theta = None):
        """Compute the joint positions required to reach a target position.
        
        An analytical solution is computed for the planar two-link manipulator.
        When two solutions exist, the solution requiring the smaller overall
        change in the joint angles from the current configuration is returned.
        
        Parameters
        ----------
        target : array_like, shape (2,)
            Desired end-effector position [x, y] in the space frame.
        theta : None
            Target end-effector orientation. Orientation targets are not
            supported for this robot and must be left unspecified.

        Returns
        -------
        numpy.ndarray, shape (2,)
            The joint positions [q1, q2] in radians.
            
        Raises
        ------
        ValueError
            If theta is specified, or if the target position lies outside the
            reachable workspace.
        """

        if theta is not None:
            raise ValueError(
                "Orientation targets are not supported for this robot!"
            )
            
        x, y = target

        dist = np.hypot(x, y)
        inner = abs(self.l1 - self.l2)
        outer = self.l1 + self.l2
        
        # No solution.
        if dist < inner or dist > outer:
            raise ValueError(
                "No inverse kinematics solution for requested tip position!"
            )
        
        gamma = np.atan2(y, x)
        
        # Outer workspace boundary
        if np.isclose(dist, outer):
            return np.array([gamma, 0.0])
        
        # Inner workspace boundary
        if np.isclose(dist, inner):

            if np.isclose(self.l1, self.l2):
                
                # The target is the origin. q1 is arbitrary.
                return np.array([self.joints[0].q, np.pi])
            
            # The direction of the resulting vector is determined by
            # whichever link is longer.
            q1 = gamma if self.l1 > self.l2 else gamma + np.pi
            return np.array([q1, np.pi])
        
        # Two solutions inside the workspace.
        alpha = np.acos(
            (self.l1**2 + dist**2 - self.l2**2)
            / (2 * self.l1 * dist)
        )
        beta = np.acos(
            (self.l1**2 + self.l2**2 - dist**2)
            / (2 * self.l1 * self.l2)
        )

        sol1 = np.array([gamma - alpha, np.pi - beta])
        sol2 = np.array([gamma + alpha, beta - np.pi])
        
        # Select the solution with the smaller overall angle difference
        def angle_diff(a, b):
            return np.arctan2(np.sin(a - b), np.cos(a - b))

        qs = np.array([joint.q for joint in self.joints[:2]])        

        d1 = np.linalg.norm(angle_diff(sol1, qs))
        d2 = np.linalg.norm(angle_diff(sol2, qs))

        return sol1 if d1 < d2 else sol2