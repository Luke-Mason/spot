from enum import Enum
import string


class Location(Enum):
    room_1 = ["ROOM ONE", "ROOM 1"]
    room_2 = ["ROOM TWO", "ROOM 2"]
    room_3 = ["ROOM THREE", "ROOM 3"]

class Target(Enum):
    luke = "LUKE"

class Task(Enum):
    find = "FIND"
    go_to = "GO TO"
    check = "CHECK"
    awake_call = "HEY SPOT"

class Listen(Enum):
    response = 4
    awake = 2
    command = 4

class Say(Enum):
    im_listening = 0




class Command():
  def __init__(self, task: Task = None, target: Target = None, location: Location = None):
    self.task = task
    self.target = target
    self.location = location

  def get_name(self):
    return self.name

  def get_location(self):
    return self.location

  def get_target(self):
    return self.target
