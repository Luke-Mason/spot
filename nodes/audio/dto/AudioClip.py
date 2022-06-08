from dataclasses import dataclass


@dataclass
class AudioClip():
    hz: int
    data: object