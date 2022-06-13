from enum import Enum
import playsound
import random
import rospy


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

class Say():
  def __init__(self, msg, path_to_media: str, audio_files, listen: Listen):
    self.msg = msg
    self.path_to_media = path_to_media
    self.audio_paths = audio_files
    self.listen = listen

  def perform(self, listen_pub: rospy.Publisher = None):
    if len(self.audio_paths) > 0:
      num = random.randint(0, len(self.audio_paths) - 1)
      playsound.playsound(self.path_to_media + "/" + str(self.say.value[num]), False)

    if listen_pub is not None:
      listen_pub.publish(self.listen.name)

class SayImListening(Say):
  def __init__(self, path_to_media: str):
    super().__init__("Yes?", path_to_media, ["yes?.mp3"], Listen.command)

class SayImSearching(Say):
  def __init__(self, path_to_media: str):
    super().__init__("Okay, Searching :)", path_to_media, ["ok_searching.mp3"])

class SayImGoing(Say):
  def __init__(self, path_to_media: str):
    super().__init__("Okay, going now!", path_to_media, ["ok_going_now.mp3"])

class SayStopping(Say):
  def __init__(self, path_to_media: str):
    super().__init__("Stopped", path_to_media, ["stopping_now.mp3"])

class SayIdoNotUnderstand(Say):
  def __init__(self, path_to_media: str, listen: Listen):
    super().__init__("I don't understand, please say it again.", path_to_media, ["i_do_not_understand.mp3"], listen)

class SayNothing(Say):
  def __init__(self):
    super().__init__("", [])


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

  def run(self):
    pass

  def perform(self, path_to_media, listen_pub: rospy.Publisher):
    self.run()
    self.saying.perform(path_to_media, listen_pub)
    

class FindLuke(Command):
  def __init__(self):
    super().__init__(Task.find, Target.luke, Sayings.ok_searching)

class GoToRoom1(Command):
  def __init__(self):
    super().__init__(Task.go_to, Target.room_1, Sayings.ok_going)
    
class Stop(Command):
  def __init__(self):
    super().__init__(Task.stop, say=Sayings.stopping)


class Commands(Enum):
  go_to_room_1 = GoToRoom1()
  find_luke = FindLuke()
  stop = Stop()

