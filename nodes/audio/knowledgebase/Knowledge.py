



# LOCATIONS
class NamedLocations():
    room_1 = NamedLocation("room 1", ["ROOM ONE", "ROOM 1"]),
    room_2 = NamedLocation("room 2", ["ROOM TWO", "ROOM 2"]),
    room_3 = NamedLocation("room 3", ["ROOM THREE", "ROOM 3"])
    
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

