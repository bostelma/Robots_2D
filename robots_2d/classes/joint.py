from dataclasses import dataclass
from enum import Enum

from spatialmath import SE3, Twist3

from .frame import Frame


class JointType(Enum):
    """Types of joints supported by the robot model.

    Each joint type defines the number of degrees of freedom it contributes
    to the robot.
    """

    FIXED = "fixed"
    REVOLUTE = "revolute"

    @property
    def dof(self) -> int:
        """Return the number of degrees of freedom for the joint type."""
        return {
            JointType.FIXED: 0,
            JointType.REVOLUTE: 1,
        }[self]


@dataclass
class Joint:
    """A kinematic joint connecting two coordinate frames.
    
    Parameters
    ----------
    name : str
        The name identifying the joint.
    parent : Frame
        The parent frame connected to the joint.
    child : Frame
        The child frame connected to the joint.
    type : JointType
        The type of the joint.
    screw_axis : spatialmath.Twist3
        The screw axis of the joint, expressed in the space frame.
    q : float, optional
        The current joint position, expressed as an angle for a revolute joint
        or a displacement for a prismatic joint. Defaults to 0.0.
    
    Attributes
    ----------
    name : str
        The name identifying the joint.
    parent : Frame
        The parent frame connected to the joint.
    child : Frame
        The child frame connected to the joint.
    type : JointType
        The type of the joint.
    screw_axis : spatialmath.Twist3
        The screw axis of the joint, expressed in the space frame.
    q : float
        The current joint position.
    
    Methods
    -------
    transform()
        Return the joint transformation for the current joint position.
    """

    name: str
    parent: Frame
    child: Frame
    type: JointType
    screw_axis: Twist3
    q: float = 0.0

    def transform(self) -> SE3:
        """Return the transformation induced by the joint.
        
        Returns
        -------
        spatialmath.SE3
            The SE(3) transformation corresponding to the current joint
            position q.
        """

        return self.screw_axis.exp(self.q)