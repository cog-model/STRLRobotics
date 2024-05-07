from dataclasses import dataclass, field
from enum import auto, Enum
from typing import List
import json


class Items(Enum):
    tomato = auto
    cucumber = auto()
    toy_cat = auto()
    cat = auto()
    stuffed_cat = auto()
    orange_cat = auto()
    cube = auto()
    toy_box = auto()
    red_box = auto()
    orange_box = auto()
    apple = auto()
    pear = auto()
    banana = auto()
    socks = auto()
    t_shirt = auto()
    pepper = auto()
    bottle = auto()
    can = auto()
    green_sweet_pepper = auto()


@dataclass
class Subtask:
    text: str = ""

    def to_json(self) -> List:
        return self.text


@dataclass
class Task:
    goal_eng: str = ""
    request: str = ""
    explanation: str = ""
    items: List[Items] = field(default_factory=list)
    subtasks: List[Subtask] = field(default_factory=list)
    valid_plan: bool = True
    clean_room: bool = False

    def to_json(self, id: int = 0):
        out = {}
        out["task_type"] = "test"
        out["plan_id"] = id
        out["goal_eng"] = self.goal_eng
        out["items"] = [item.name.lower().replace("_", " ") for item in self.items]
        out["request"] = self.request
        out["valid_plan"] = self.valid_plan
        out["clean_room"] = self.clean_room

        plan = []
        for subtask in self.subtasks:
            plan.append(subtask.to_json())
        out["plan"] = plan

        return out


tests: List[Task] = [
    Task("Make sure the room is spotless for the visitors.", clean_room=True),
    Task("Organize your belongings and straighten up the room, please.", clean_room=True),
    Task(
        "We need to vacuum and dust the entire space before the party.", clean_room=True
    ),
    Task("I think it's time to declutter and clean your room.", clean_room=True),
    Task("Make sure the room is spotless for the visitors.", clean_room=True),
    Task(
        "Please check the kitchen appliances, but leave the room as it is.",
        clean_room=False,
    ),
    Task(
        "It's essential to fix the lights in the hallway before guests arrive.",
        clean_room=False,
    ),
    Task(
        "Review the documents on the desk, and then we can discuss the project.",
        clean_room=False,
    ),
    Task("Take out the trash and water the plants in the room.", clean_room=False),
    Task("Clean room", clean_room=True),
    Task("Clean this room", clean_room=True),
    Task("Clean up all the stuff in the room", clean_room=True),
]
# You have a home robot equipped with three actions: move_to(object, location), pick(object, location), and put(object, location). The robot is capable of navigating within a defined area, picking up objects at specific locations, and placing them at other locations.
dataset = []
for i, task in enumerate(tests):
    dataset.append(task.to_json(i))
    print(task.goal_eng)

with open("./data/hlp/clean_room.json", "w", encoding="utf8") as f:
    json.dump(dataset, f, indent=4, ensure_ascii=False)
