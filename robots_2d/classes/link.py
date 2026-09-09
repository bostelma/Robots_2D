from dataclasses import dataclass

from .frame import Frame


@dataclass
class Link:

    name: str       # The name of the link
    frame: Frame    # The frame attached to this link
    length: float   # The length of the link