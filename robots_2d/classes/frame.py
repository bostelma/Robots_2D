from dataclasses import dataclass, field

from spatialmath import SE3


@dataclass
class Frame:
    """A named coordinate frame represented by an SE(3) transformation.
    
    Parameters
    ----------
    name : str
        The name identifying the coordinate frame.
    M : spatialmath.SE3, optional
        Home pose of the frame expressed with respect to the space frame.
        Defaults to the identity transformation.
        
    Attributes
    ----------
    name : str
        The name identifying the coordinate frame.
    M : spatialmath.SE3
        Home pose of the frame expressed with respect to the space frame.
    """
    
    name: str 
    M: SE3 = field(
        default_factory = SE3
    )