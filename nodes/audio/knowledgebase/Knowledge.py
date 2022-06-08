from nodes.audio.knowledgebase.Location import Location
from audio.knowledgebase.Task import Task


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

