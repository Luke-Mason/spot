from dataclasses import dataclass
import string
from aiil_workspace.noetic_workspace.src.SPOT.nodes.audio.knowledgebase.NamedLocation import NamedLocation
from aiil_workspace.noetic_workspace.src.SPOT.nodes.audio.knowledgebase.Target import Target
from aiil_workspace.noetic_workspace.src.SPOT.nodes.audio.knowledgebase.Task import Task
from std_msgs.msg import String


@dataclass
class Command():
  def __init__(self, task: Task = None, target: Target = None, location: NamedLocation = None):
    self.task = task
    self.target = target
    self.location = location

  def get_name(self):
    return self.name

  def get_location(self):
    return self.location

  def get_target(self):
    return self.target
