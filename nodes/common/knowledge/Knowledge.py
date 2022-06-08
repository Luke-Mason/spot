import string
from knowledge.msg.Say import Say

# LOCATIONS
class KnownLocations():
    room_1 = Location("room 1", ["ROOM ONE", "ROOM 1"]),
    room_2 = Location("room 2", ["ROOM TWO", "ROOM 2"]),
    room_3 = Location("room 3", ["ROOM THREE", "ROOM 3"])
    
    all_known_locations = [room_1, room_2, room_3]



# TARGETS
class KnownTargets():
    luke = "LUKE"
    
    all_known_targets = [luke]

# TASKS
class KnownTasks():
    # Move task
    go_to = Task("GO TO")

    # Explore task
    find = Task("FIND")
    check = Task("CHECK")
    
    all_known_tasks = [go_to, find, check]
    
    awake_call = "HEY SPOT"

class Sayings():
    im_listening = Say("yes? uhuh? what is it?")


class Location():
  def __init__(self, id: string, aliases) -> None:
      self.id = id
      self.aliases = aliases

class Target():
    def __init__(self, name: string) -> None:
        self.name = name
    
    def get_name(self):
        return self.name


class Task():
    def __init__(self, name: string) -> None:
        self.name = name
    
    def get_name(self):
        return self.name