from dataclasses import dataclass

from .frame import Frame


@dataclass
class Link:
    """A rigid link in a kinematic chain.
    
    Parameters
    ----------
    name : str
        The name identifying the link.
    frame : Frame
        The coordinate frame attached to the link.
    length : float
        The length of the link in metres.
        
    Attributes
    ----------
    name : str
        The name identifying the link.
    frame : Frame
        The coordinate frame attached to the link.
    length : float
        The length of the link in metres.
    """

    name: str
    frame: Frame
    length: float