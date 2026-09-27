from abc import ABC, abstractmethod
from dataclasses import dataclass, field

import numpy as np
import sympy as sp
from spatialmath import SE3

from robots_2d.classes.frame import Frame
from robots_2d.classes.joint import Joint, JointType
from robots_2d.classes.link import Link


@dataclass
class Robot(ABC):
    """Abstract base class for a kinematic robot model.
    
    The robot is represented as a collection of coordinate frames, joints, and
    links. Joint motion is described using screw axes expressed in the space
    frame, and kinematic quantities are computed using the product of
    exponentials formulation.
    
    Parameters
    ----------
    frames : list of Frame, optional
        Coordinate frames belonging to the robot. Defaults to an empty list.
    joints : list of Joint, optional 
        Joints belonging to the robot, ordered from the base towards the end
        effector. Defaults to an empty list.
    links : list of Link, optional
        Links belonging to the robot. Defaults to an empty list.
        
    Attributes
    ----------
    frames : list of Frame
        Coordinate frames belonging to the robot.
    joints : list of Joint
        Joints belonging to the robot, ordered from the base towards the end
        effector.
    links : list of Link
        Links belonging to the robot.
        
    Notes
    -----
    The kinematic methods in this class assume a planar, open-chain mechanism
    with joints ordered from the base towards the end effector. Fixed joints are
    included in the kinematic chain but do not contribute degrees of freedom or
    columns to the Jacobian.
    """

    frames: list[Frame] = field(default_factory=list)
    joints: list[Joint] = field(default_factory=list)
    links: list[Link] = field(default_factory=list)

    # --------------------------------------------------------------------------
    # General properties
    # --------------------------------------------------------------------------

    def dof(self) -> int:
        """Compute the robot's degree of freedom using Grübler's formula.

        Returns
        -------
        int
            The degree of freedom of the robot.

        Notes
        -----
        Grübler's formula assumes that the constraints of the mechanical system
        are independent.
        """

        # The number of bodies including ground
        N = 1 + len(self.links)

        # The number of joints
        J = len(self.joints)

        # 6 for spatial bodies and 3 for planar
        m = 3

        # DOF for each joint
        f = [joint.type.dof for joint in self.joints]

        # Compute Grübler's formula
        return m * (N - 1 - J) + sum(f)

    # --------------------------------------------------------------------------
    # Robot interaction
    # --------------------------------------------------------------------------

    def move_joints(self, qs):
        """Set the positions of all movable joints.
        
        Joint positions are assigned to movable joints in the order in which
        they occur in self.joints. Fixed joints are ignored. 
        
        Parameters
        ----------
        qs : array_like, shape (N,)
            Joint positions to assign to the movable joints. For revolute
            joints, positions are interpreted as angles; for prismatic joints,
            they are interpreted as displacements.
            
        Raises
        ------
        ValueError
            If the number of supplied joint positions does not match the number
            of movable joints.
        """

        qs = np.asarray(qs)

        movable_joints = [
            joint for joint in self.joints
            if joint.type != JointType.FIXED
        ]

        if qs.shape[0] != len(movable_joints):
            raise ValueError(
                "Number of given joint values and movable joints don't match!"
            )

        for joint, q in zip(movable_joints, qs):
            joint.q = q

    # --------------------------------------------------------------------------
    # Forward kinematics
    # --------------------------------------------------------------------------
    
    def forward_kinematics(self):
        """Compute the forward kinematics of the open-chain mechanism.
        
        The kinematics are evaluated using the product of exponentials formula
        in the space frame. Joints are processed in the order in which they
        occur in self.joints.
        
        Returns
        -------
        dict[str, spatialmath.SE3]
            A mapping from child frame names to their poses expressed in the
            space frame.
            
        Notes
        -----
        This implementation assumes that the joints are ordered from the base
        towards the end effector and that the mechanism forms an open chain.
        The pose of each child frame is computed as the cumulative joint
        transformation multiplied by the frame's pose M in the space frame.
        """

        poses = {}
        T = SE3()

        # Evaluate the product of exponentials formula in space frame
        for joint in self.joints:

            T = T * joint.transform()

            poses[joint.child.name] = T * joint.child.M

        return poses

    # --------------------------------------------------------------------------
    # Inverse kinematics
    # --------------------------------------------------------------------------

    @abstractmethod
    def inverse_kinematics(self, target):
        """Compute the joint positions required to reach a target position.
        
        Parameters
        ----------
        target : array_like, shape (2,)
    
        Returns
        -------
        numpy.ndarray, shape (N,)
            Joint positions that bring the end effector to the target position,
            where N is the number of movable joints.    
        """

    def inverse_kinematics_num(self, target, tol = 0.1, max_iterations = 100):
        """Compute the joint positions required to reach a target position.
        
        The inverse kinematics problem is solved numerically using a
        Newton-Raphson iteration based on the body Jacobian. Only the x- and
        y-components of the end-effector position are considered. The robot's
        joint configuration is restored to its original state before the method
        returns.
        
        Parameters
        ----------
        target : array_like, shape (2,)
            Desired end-effector position [x, y] in the space frame.
        tol : float, optional
            Convergence tolerance for the Euclidean norm of the x/y position
            error. Defaults to 0.1.
        max_iterations : int, optional
            Maximum number of Newton-Raphson iterations. Defaults to 100.
            
        Returns
        -------
        numpy.ndarray, shape (N,)
            Joint positions that bring the end effector to the target position,
            where N is the number of movable joints.
        
        Raises
        ------
        RuntimeError
            If the solver fails to converge within max_iterations.
            
        Notes
        -----
        Only the translational x/y components of the body Jacobian are used for
        the Newton-Raphson update. Fixed joints are not included in the returned
        joint configuration. The method temporarily modifies the robot's joint
        configuration during the iteration but restores the original
        configuration before returning.
        """
        
        # Save the current joint configuration
        q_original = np.array([
            joint.q for joint in self.joints
            if joint.type != JointType.FIXED
        ])

        q = q_original.copy()

        # Desired end-effector pose
        Tsd = SE3.Trans(target[0], target[1], 0)

        # Limit the number of iterations
        for _ in range(max_iterations):

            # Current end-effector pose
            Tsb = self.forward_kinematics()['link_end']

            # Body-frame pose error
            Vb = (Tsb.inv() @ Tsd).log()

            # Only consider x/y translational error
            v = Vb[:2, 3]

            # Check convergence
            if np.linalg.norm(v) <= tol:
                break

            # Body Jacobian
            Jb = self.body_jacobian()

            # Only x/y translational component
            Jv = Jb[:2, :]

            # Newton-Raphson update
            q += np.linalg.pinv(Jv) @ v

            # Apply updated joint configuration
            self.move_joints(q)

        else:
            raise RuntimeError(
                "Inverse kinematics failed to converge."
            )

        # Restore original robot configuration
        self.move_joints(q_original)

        return q

    # --------------------------------------------------------------------------
    # Jacobians
    # --------------------------------------------------------------------------

    def space_jacobian(self):
        """Compute the space Jacobian of the mechanism.
        
        The Jacobian is computed using the screw axes of the joints and the
        product of exponentials formulation. Each screw axis is transformed
        according to the current configuration and expressed in the space frame.
        
        Returns
        -------
        numpy.ndarray, shape (6, N)
            The space Jacobian, where N is the number of movable joints. Each
            column contains the spatial screw associated with one movable joint.
            
        Notes
        -----
        This implementation assumes that the joints are ordered from the base
        towards the end effector and that the mechanism forms an open chain.
        Fixed joints do not contribute columns to the Jacobian.
        """

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

    def body_jacobian(self):
        """Compute the body Jacobian of the mechanism.
        
        The body Jacobian is obtained by transforming the space Jacobian into
        the end-effector frame.
        
        Returns
        -------
        numpy.ndarray, shape (6, N)
            The body Jacobian, where N is the number of movable joints. Each
            column contains the screw associated with one movable joint,
            expressed in the end-effector frame.
            
        Notes
        -----
        This implementation assumes that the joints are ordered from the
        base towards the end effector and that the mechanism forms an open
        chain. Fixed joints do not contribute columns to the Jacobian.
        """

        Js = self.space_jacobian()

        poses = self.forward_kinematics()
        Tsb = poses['link_end']

        Jb = Tsb.inv().Ad() @ Js

        return Jb

    # --------------------------------------------------------------------------
    # Manipulability
    # --------------------------------------------------------------------------

    def manipulability_ellipse(self):
        """Compute the planar manipulability ellipse of the end effector.
        
        The ellipse is computed from the translational part of the body Jacobian
        and expressed in the space frame. Its center is given by the current x/y
        position of the end effector.
        
        Returns
        -------
        center : numpy.ndarray, shape (2,)
            The x/y position of the end effector in the space frame.
        width : float
            The full length of the ellipse's major axis.
        height : float
            The full length of the ellipse's minor axis.
        angle : float The orientation of the major axis in degrees, measured
        counterclockwise from the positive x-axis of the space frame.
        
        Notes
        -----
        The manipulability ellipse is computed from the planar translational
        Jacobian Jxy. The axis lengths are proportional to the square roots of
        the eigenvalues of the manipulability matrix. The ellipse is expressed
        in the space frame, while the translational body Jacobian is first
        rotated from the end-effector frame into the space frame.
        """

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

    # --------------------------------------------------------------------------
    # Trajectory generation
    # --------------------------------------------------------------------------
    
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
            qs.append(self.inverse_kinematics(X))

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
            self.inverse_kinematics(pos)
                for pos in via_points
        ]

        via_points_joint_space = np.array(via_points_joint_space)

        return self.trajectory_via_points_joint_space(
            via_points_joint_space,
            Ts,
            f
        )