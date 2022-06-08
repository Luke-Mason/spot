from dataclasses import dataclass


@dataclass
class AudioClip():
    hz: int # sample rate
    data: object