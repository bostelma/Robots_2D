from abc import ABC, abstractmethod
from dataclasses import dataclass, field

import numpy as np
from spatialmath import SE3

from robots_2d.classes.frame import Frame
from robots_2d.classes.joint import Joint, JointType
from robots_2d.classes.link import Link


@dataclass
class Robot(ABC):

    frames: list[Frame] = field(default_factory=list)
    joints: list[Joint] = field(default_factory=list)
    links: list[Link] = field(default_factory=list)

    def move_joints(self, *qs):

        joint_angles = list(qs)

        i = 0
        for joint in self.joints:

            if joint.type == JointType.FIXED:
                continue

            joint.q = joint_angles[i]
            i += 1

        if i != len(joint_angles):

            raise ValueError(
                "Number of given joint angles and movable joints don't match!"
            )

    def forward_kinematics(self):

        # NOTE: This forward kinematics only holds for orderd joint
        #       in an open-chain configuration!

        poses = {}
        T = SE3()

        # Evaluate the product of exponentials formula in space frame
        for joint in self.joints:

            T = T * joint.transform()

            poses[joint.child.name] = T * joint.child.M

        return poses

    @abstractmethod
    def inverse_kinematics(self, x, y):
        pass

    def space_jacobian(self) -> np.ndarray:

        # NOTE: This jacobian computation only holds for orderd joint
        #       in an open-chain configuration!

        J_columns = []

        T = SE3()

        for joint in self.joints:

            # Fixed joints do not add DOF
            if joint.type == JointType.FIXED:
                T = T * joint.transform()
                continue

            # Transform the screw axis of this joint
            # into the current space configuration.
            S_current = T.Ad() @ joint.screw_axis

            J_columns.append(S_current)

            # Accumulate transformation for subsequent joints
            T = T * joint.transform()

        return np.column_stack(J_columns)

    def body_jacobian(self) -> np.ndarray:

        # NOTE: This jacobian computation only holds for orderd joint
        #       in an open-chain configuration!

        Js = self.space_jacobian()

        poses = self.forward_kinematics()
        Tsb = poses['link_end']

        Jb = Tsb.inv().Ad() @ Js

        return Jb

    def manipulability_ellipse(self) -> np.ndarray:

        # End effector pose
        poses = self.forward_kinematics()
        T = poses["link_end"]

        # Position of end effector in world frame
        center = T.t[:2]

        # Body Jacobian
        Jb = self.body_jacobian()

        # Translational body Jacobian
        Jb_xy = Jb[:2, :]

        # Rotate into space frame
        R = T.R[:2, :2]
        Jxy = R @ Jb_xy

        # Manipulability matrix
        A = Jxy @ Jxy.T

        # Eigen-decomposition
        eigenvalues, eigenvectors = np.linalg.eigh(A)

        # Sort largest -> smallest
        order = np.argsort(eigenvalues)[::-1]
        eigenvalues = eigenvalues[order]
        eigenvectors = eigenvectors[:, order]

        # Full axis lengths for matplotlib
        width = 2 * np.sqrt(eigenvalues[0])
        height = 2 * np.sqrt(eigenvalues[1])

        # Orientation of major axis
        major_axis = eigenvectors[:, 0]

        angle = np.degrees(
            np.arctan2(major_axis[1], major_axis[0])
        )

        return center, width, height, angle