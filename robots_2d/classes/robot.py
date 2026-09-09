from abc import ABC
from dataclasses import dataclass, field

from spatialmath import SE3

from robots_2d.classes.frame import Frame
from robots_2d.classes.joint import Joint
from robots_2d.classes.link import Link


@dataclass
class Robot(ABC):

    frames: list[Frame] = field(default_factory=list)
    joints: list[Joint] = field(default_factory=list)
    links: list[Link] = field(default_factory=list)

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