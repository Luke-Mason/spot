from enum import Enum
import random
import rospy
import simpleaudio as sa
import pydub


class Location(Enum):
    room_1 = ["ROOM ONE", "ROOM 1"]
    room_2 = ["ROOM TWO", "ROOM 2"]
    room_3 = ["ROOM THREE", "ROOM 3"]

class Target(Enum):
    luke = 0
    room_1 = 1

class Task(Enum):
    find = ["FIND", "FI ND", "IND"]
    go_to = ["GO", "GOO", "G OO", "G O"]
    # check = ["CHECK"]
    awake = ["SPOT", "SPO OT", "SPOOT"]
    stop = ["STOP", "ST OP", "S TOP", "STO P"]

class Listen(Enum):
    response = 4
    awake = 2
    command = 4
    paused = 0

class Say():
  def __init__(self, msg: str, audio_files, listen: Listen):
    self.msg = msg
    self.audio_files = audio_files
    self.listen = listen


# TODO Make these classes from a config file
class SayImListening(Say):
  def __init__(self):
    super().__init__("Yes?", ["yes.wav", "huh.wav", "uhuh.wav", "yes_what_is_it.wav"], Listen.command)

class SayImSearching(Say):
  def __init__(self):
    super().__init__("Okay, Searching :)", ["ok_searching.wav", "finding_them_now.wav"], Listen.awake)

class SayImGoing(Say):
  def __init__(self):
    super().__init__("Okay, going now!", ["ok_going.wav", "moving_now.wav", "ok_on_my_way.wav"], Listen.awake)

class SayStopping(Say):
  def __init__(self):
    super().__init__("Stopped", ["stopping_now.wav", "stopping.wav", "okay.wav"], Listen.awake)

class SayIdoNotUnderstand(Say):
  def __init__(self): # , listen: Listen
    super().__init__("I don't understand, please say it again.", 
    ["sorry_can_you_say_that_again.wav", "sorry_what_was_that.wav", "what_did_you_say.wav", "i_did_not_catch_that.wav"], None)

class SayNothing(Say):
  def __init__(self):
    super().__init__("", [], Listen.awake)


# TODO make custom ros msg instead of this enum facade that is using String. Make Say msg.
class Sayings(Enum):
  im_listening = SayImListening()
  stopping = SayStopping()
  ok_going = SayImGoing()
  ok_searching = SayImSearching()
  i_do_not_understand = SayIdoNotUnderstand()
  nothing = SayNothing()


class Command():
  def __init__(self, task: Task = None, target: Target = None, saying: Sayings = None):
    self.task = task
    self.target = target
    self.saying = saying

  def get_task_enum(self):
    return self.task

  def perform(self):
    pass

  def run(self, say_pub: rospy.Publisher):
    self.perform()
    say_pub.publish(self.saying.name)
    

class FindLuke(Command):
  def __init__(self):
    super().__init__(Task.find, Target.luke, Sayings.ok_searching)

  def perform(self):
    rospy.loginfo("PERFORMING FIND LUKE")

class GoToRoom1(Command):
  def __init__(self):
    super().__init__(Task.go_to, Target.room_1, Sayings.ok_going)
  
  def perform(self):
    rospy.loginfo("PERFORMING GO TO ROOM 1")

class Stop(Command):
  def __init__(self):
    super().__init__(Task.stop, saying=Sayings.stopping)

  def perform(self):
    rospy.loginfo("PERFORMING STOP")

# TODO make custom ros msg instead of this enum facade that is using String. Make Command msg
class Commands(Enum):
  go_to_room_1 = GoToRoom1()
  find_luke = FindLuke()
  stop = Stop()

