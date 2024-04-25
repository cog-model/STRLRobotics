import enum
from dataclasses import dataclass


class Instruction(enum.Enum):
    PUT = 1
    PICK_UP = 2
    MOVE_TO = 3
    RECOGNIZE = 4
    FIND = 5
    STOP = 6

@dataclass
class LLMTask:
    type: Instruction
    object: str
    location: str

@dataclass   
class LLMTask_object:
    type: Instruction
    object: str

@dataclass
class Command:
    type: Instruction
