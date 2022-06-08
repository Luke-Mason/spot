from enum import Enum
import string


Await = Enum("Await", "awake response command")

class Listen():
    def __init__(self, awaitType: Await, duration: int):
        self.awaitType = awaitType
        self.duration = duration


class ListenerStatus():
    response = Listen(Await.response, 4)
    awake = Listen(Await.awake, 2)
    command = Listen(Await.command, 5)