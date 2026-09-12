from dataclasses import dataclass
from enum import Enum

from spatialmath import SE3, Twist3

from .frame import Frame


class JointType(Enum):
    FIXED = "fixed"
    REVOLUTE = "revolute"


@dataclass
class Joint:

    name: str               # Name of the joint
    parent: Frame           # The parent frame
    child: Frame            # The child frame
    type: JointType         # The type of joint
    screw_axis: Twist3      # The screw axis of the joint
    q: float = 0.0          # The joint angle

    def transform(self) -> SE3:
        # Joint transformation for the current q
        return self.screw_axis.exp(self.q)