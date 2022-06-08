from dataclasses import dataclass
from nodes.audio.knowledgebase.Location import Location
from audio.knowledgebase.Target import Target
from audio.knowledgebase.Task import Task


@dataclass
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
