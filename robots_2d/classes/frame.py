from dataclasses import dataclass, field

from spatialmath import SE3


@dataclass
class Frame:

    name: str                   # The name of the frame
    M: SE3 = field(             # The frame expressed in the space frame
        default_factory=SE3
    )