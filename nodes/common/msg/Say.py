from dataclasses import dataclass
import string


@dataclass
class Say():
    def __init__(self, phrase: string):
        self.phrase = phrase

    def get_phrase(self):
        return self.phrase