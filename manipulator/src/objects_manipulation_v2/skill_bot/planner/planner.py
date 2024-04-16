#std (pip)
from enum import Enum, auto
from typing import List

#custom
from skill_bot.planner.ctx_manager import Ctx
from skill_bot.robot.husky import Robot
from skill_bot.primitives import entity


class EProperty(Enum):
    ID = auto()
    POSITION = auto()
    COLOR = auto()
    TYPE = auto()
    RANDOM = auto()



class Ur5Planner:
    def __init__(self, ctx : Ctx) -> None:
        self.target : entity.RealObject 
    
    
    
    def select_target(by : EProperty.value, property : object):
        if by == EProperty.ID:
            pass
        elif by == EProperty.POSITION:
            pass
        elif by == EProperty.COLOR:
            pass
        elif by == EProperty.TYPE:
            pass
        elif by == EProperty.RANDOM:
            pass
    
    
    
    def grasp_plan()->List[List[float]]:
        pass
    
    
    
    def place_plan()->List[List[float]]:
        pass


