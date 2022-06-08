import string


class Target():
    def __init__(self, name: string) -> None:
        self.name = name
    
    def get_name(self):
        return self.name
        