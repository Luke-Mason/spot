from enum import Enum
import string


class Location(Enum):
    room_1 = ["ROOM ONE", "ROOM 1"]
    room_2 = ["ROOM TWO", "ROOM 2"]
    room_3 = ["ROOM THREE", "ROOM 3"]

class Target(Enum):
    luke = "LUKE"

class Calls(Enum):
    find = ["FIND", "FI ND", "IND"]
    go_to = ["GO", "GOO", "G OO", "G O"]
    # check = ["CHECK"]
    awake = ["SPOT", "SPO OT", "SPOOT"]
    stop = ["STOP", "ST OP", "S TOP", "STO P"]

class Listen(Enum):
    response = 4
    awake = 2
    command = 4

class Say(Enum):
    im_listening = ["../media/yes?.mp3"]
    ok_searching = ["../media/ok_searching.mp3"]
    ok_going = ["../media/ok_going_now.mp3"]
    did_not_understand = ["../media/i_do_not_understand.mp3"]
    stopping = ["stopping_now.mp3"]
    nothing = []




class Command():
  def __init__(self, task: Calls = None, target: Target = None, location: Location = None):
    self.task = task
    self.target = target
    self.location = location

  def get_name(self):
    return self.name

  def get_location(self):
    return self.location

  def get_target(self):
    return self.target
