from abc import ABC, abstractmethod
from dataclasses import dataclass, field

import sympy as sp
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

    # ------------------------------------------------------------
    # Trajectory Generation
    # ------------------------------------------------------------
    
    def time_scaling(self, T: float, f: float, method: str):

        match method:

            case 'linear':

                # Linear time scaling
                return np.linspace(0.0, 1.0, T * f)

            case 'poly3':

                # Third-order polynomial time scaling
                t = np.linspace(0.0, T, T * f)
                return 3 / T**2 * t**2 - 2 / T**3 * t**3

            case 'poly5':

                # Fifth-order polynomial time scaling
                t = np.linspace(0.0, T, T * f)
                return 10 / T**3 * t**3 - 15 / T**4 * t**4 + 6 / T**5 * t**5

            case m:

                raise ValueError(
                    f'Time scaling method {m} not supported!'
                )
    
    def trajectory_p2p_joint_space(
            self,
            q_start: np.ndarray,
            q_end: np.ndarray,
            T: float = 1.0,
            f: float = 50,
            time_scaling = 'linear'
        ):

        s = self.time_scaling(T, f, time_scaling)

        qs = np.vstack([
            q_start + si * (q_end - q_start) for si in s
        ])

        return qs

    def trajectory_p2p_cartesisan_space(
            self,
            X_start: np.ndarray,
            X_end: np.ndarray,
            T: float = 1.0,
            f: float = 50,
            time_scaling = 'linear'
        ):

        s = self.time_scaling(T, f, time_scaling)

        Xs = np.vstack([
            X_start + si * (X_end - X_start) for si in s
        ])

        qs = []
        for X in Xs:
            qs.append(self.inverse_kinematics(*X))

        return np.array(qs)

    def trajectory_via_points_joint_space(
            self,
            via_points: np.ndarray,
            Ts: np.ndarray,
            f: float = 50
    ):

        # Number of segments
        N = len(via_points) - 1

        # For each joint
        solutions = []

        for joint_index in range(via_points.shape[1]):

            a = sp.Matrix(
                N,
                4,
                lambda i, j: sp.Symbol(f'a_{j}_{i}')
            )
    
            equations = []

            # Positional constraints
            for j in range(N):

                equations.append(
                    sp.Eq(a[j,0] + a[j,1] * Ts[j] + a[j,2] * Ts[j]**2 + a[j,3] * Ts[j]**3, via_points[j,joint_index])
                )

                equations.append(
                    sp.Eq(a[j,0] + a[j,1] * Ts[j+1] + a[j,2] * Ts[j+1]**2 + a[j,3] * Ts[j+1]**3, via_points[j+1,joint_index])
                )

            # Velocity constraints
            equations.append(
                sp.Eq(a[0,1] + 2 * a[0,2] * Ts[0] + 3 * a[0,3] * Ts[0]**2, 0)
            )
            equations.append(
                sp.Eq(a[-1,1] + 2 * a[-1,2] * Ts[-1] + 3 * a[-1,3] * Ts[-1]**2, 0)
            )

            for j in range(N-1):
                equations.append(
                    sp.Eq(a[j,1] + 2 * a[j,2] * Ts[j+1] + 3 * a[j,3] * Ts[j+1]**2, a[j+1,1] + 2 * a[j+1,2] * Ts[j+1] + 3 * a[j+1,3] * Ts[j+1]**2)
                )
                

            # Acceleration constraints
            for j in range(N-1):
                equations.append(
                    sp.Eq(2 * a[j,2] + 6 * a[j,3] * Ts[j+1], 2 * a[j+1,2] + 6 * a[j+1,3] * Ts[j+1])
                )

            # Solve the system
            solution = sp.solve(equations, a)

            solutions.append(solution)

        # Sample qs
        ts = np.arange(Ts[0], Ts[-1], 1 / f)

        if ts[-1] < Ts[-1]:
            ts = np.append(ts, Ts[-1])

        qs = []

        for t in ts:

            # Find the segment containing t
            j = np.searchsorted(Ts, t, side="right") - 1

            # Prevent going past the final segment
            j = min(j, len(Ts) - 2)

            q = []

            for i in range(via_points.shape[1]):

                sol = solutions[i]

                qi = (
                    sol[a[j, 0]]
                    + sol[a[j, 1]] * t
                    + sol[a[j, 2]] * t**2
                    + sol[a[j, 3]] * t**3
                )

                q.append(float(qi))

            qs.append(q)

        qs = np.array(qs)

        return qs

    def trajectory_via_points_cartesian_space(
                self,
                via_points: np.ndarray,
                Ts: np.ndarray,
                f: float = 50
        ):

        via_points_joint_space = [
            self.inverse_kinematics(*pos)
                for pos in via_points
        ]

        via_points_joint_space = np.array(via_points_joint_space)

        return self.trajectory_via_points_joint_space(
            via_points_joint_space,
            Ts,
            f
        )