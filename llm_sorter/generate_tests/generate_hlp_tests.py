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
    items: List[Items] = field(default_factory=list)
    subtasks: List[Subtask] = field(default_factory=list)
    valid_plan: bool = True

    def to_json(self, id: int = 0):
        out = {}
        out["task_type"] = "test"
        out["plan_id"] = id
        out["goal_eng"] = self.goal_eng
        out["items"] = [item.name.lower().replace("_", " ") for item in self.items]
        out["request"] = self.request
        out["valid_plan"] = self.valid_plan

        plan = []
        for subtask in self.subtasks:
            plan.append(subtask.to_json())
        out["plan"] = plan

        return out


tests: List[Task] = [
    Task(
        goal_eng="put all the fruits from the floor on the table",
        items=[Items.apple, Items.pear, Items.banana],
        request="fruits",
        subtasks=[
            Subtask("Put the apple from the floor on the table."),
            Subtask("Put the pear from the floor on the table."),
            Subtask("Put the banana from the floor on the table."),
        ],
    ),
    Task(
        goal_eng="remove all the objects from the floor into the box",
        items=[Items.cat, Items.cube, Items.banana],
        request="objects",
        subtasks=[
            Subtask("Put the cat from the floor in the box."),
            Subtask("Put the cube from the floor in the box."),
            Subtask("Put the banana from the floor in the box."),
        ],
    ),
    Task(
        goal_eng="Put your socks and T-shirt in the drawer",
        items=[Items.socks, Items.t_shirt],
        request="",
        subtasks=[
            Subtask("Put socks in the drawer"),
            Subtask("Put T-shirt in the drawer"),
        ],
    ),
    Task(
        goal_eng="Move the green sweet pepper from the bedside table to the drawer, and then put an apple there too",
        items=[Items.green_sweet_pepper, Items.apple],
        request="",
        subtasks=[
            Subtask("move the green sweet pepper from the bedside table to the drawer"),
            Subtask("put the apple in the drawer"),
        ],
    ),
    Task(
        goal_eng="transfer the pepper from the table to the bedside table, and then put the stuffed cat on the table",
        items=[Items.pepper, Items.stuffed_cat],
        request="",
        subtasks=[
            Subtask("transfer the pepper from the table to the bedside table"),
            Subtask("put the stuffed cat on the table"),
        ],
    ),
    Task(
        goal_eng="Hide a bottle in the box, then throw a can from the table on the armchair",
        items=[Items.bottle, Items.can],
        request="",
        subtasks=[
            Subtask("hide a bottle in the box"),
            Subtask("throw a can from the table on the armchair"),
        ],
    ),
    Task(
        goal_eng="Put all the vegetables in the container",
        items=[Items.pepper, Items.cucumber, Items.tomato],
        request="",
        subtasks=[
            Subtask("Put the pepper in the container"),
            Subtask("Put the cucumber in the container"),
            Subtask("Put the tomato in the container"),
        ],
    ),
    Task(
        goal_eng="Put the orange cat and pepper from the floor on the table",
        items=[Items.stuffed_cat, Items.pepper],
        request="",
        subtasks=[
            Subtask("Put the stuffed cat from the floor onto the table"),
            Subtask("Put the pepper from the floor onto the table"),
        ],
    ),
]
#

dataset = []
for i, task in enumerate(tests):
    dataset.append(task.to_json(i))
    print(task.goal_eng)

# pprint(dataset)

with open("./data/hlp/hlp_tests.json", "w", encoding="utf8") as f:
    json.dump(dataset, f, indent=4, ensure_ascii=False)
#         self._conv.append_message(
#             self._conv.roles[1],
#             "Hi, I'm an intelligent assistant that helps with action planning for a robot that operates in the house.",
#         )
#         self._conv.append_message(
#             self._conv.roles[1],
#             "I will request which objects I need to recognize in order to form an action plan.\n",
#         )
#         self._conv.append_message(
#             self._conv.roles[1],
#             "Then if necessary, I will break one task into subtasks to make it easier for the robot to complete them step by step.\n",
#         )
#         self._conv.append_message(
#             self._conv.roles[0], "Task: put all the fruits from the floor on the table\n"
#         )
#         self._conv.append_message(
#             self._conv.roles[1], "List of objects to recognize: ['fruits']\n"
#         )
#         self._conv.append_message(
#             self._conv.roles[0], "List of found objects: ['apple', 'pear', 'banana']\n"
#         )
#         self._conv.append_message(
#             self._conv.roles[1],
#             "1. Put the apple from the floor on the table.\n2. Put the pear from the floor on the table.\n3. Put the banana from the floor on the table.\n",
#         )
#         # Example all objects
#         self._conv.append_message(
#             self._conv.roles[0],
#             "Task: remove all the objects from the floor into the box\n",
#         )
#         self._conv.append_message(
#             self._conv.roles[1], "List of objects to recognize: ['objects']\n"
#         )
#         self._conv.append_message(
#             self._conv.roles[0], "List of found objects: ['cat', 'cube', 'socks']\n"
#         )
#         self._conv.append_message(
#             self._conv.roles[1],
#             "1. Remove cat from the floor into the box.\n2. Remove cube from the floor into the box.\n3. Remove socks from the floor into the box.\n",
#         )
#         # # Example red objects
#         self._conv.append_message(
#             self._conv.roles[0], "Task: hide all red objects inside drawer\n"
#         )
#         self._conv.append_message(
#             self._conv.roles[1], "List of objects to recognize: ['red objects']\n"
#         )
#         self._conv.append_message(
#             self._conv.roles[0],
#             "List of found objects: ['red cube', 'red cat', 'red apple']\n",
#         )
#         self._conv.append_message(
#             self._conv.roles[1],
#             "1. Hide red cube inside drawer.\n2. Hide red cat inside drawer.\n3. Hide red apple inside drawer.\n",
#         )
#         # Example one soda can with several objects
#         self._conv.append_message(
#             self._conv.roles[0], "Task: get to the bedside table, then get to the cat\n"
#         )
#         self._conv.append_message(
#             self._conv.roles[1],
#             "List of objects to recognize: ['bedside table', 'cat']\n",
#         )
#         self._conv.append_message(
#             self._conv.roles[0], "List of found objects: ['bedside table', 'cat']\n"
#         )
#         self._conv.append_message(
#             self._conv.roles[1], "1. Move to the nightstand.\n2. Move to the nightstand\n"
#         )
#         self._conv.append_message(
#             self._conv.roles[0], "Task: Put all the apples in the drawer\n"
#         )
#         self._conv.append_message(
#             self._conv.roles[1], "List of objects to recognize: ['apples']\n"
#         )
#         self._conv.append_message(
#             self._conv.roles[0], "List of found objects: ['apple 1', 'apple 2']\n"
#         )
#         self._conv.append_message(
#             self._conv.roles[1],
#             "1. Put the apple 1 in the drawer.\n2. Put the apple 2 in the drawer\n",
#         )
#         self._conv.append_message(
#             self._conv.roles[0],
#             "How would you remove all cubes from the armchair to the drawer?\n",
#         )
#         self._conv.append_message(
#             self._conv.roles[1], "List of objects to find: ['cubes']\n"
#         )
#         self._conv.append_message(
#             self._conv.roles[0],
#             "List of found objects: ['red cube', 'blue cube', 'green cube']\n",
#         )
#         self._conv.append_message(
#             self._conv.roles[1],
#             "1. Remove red cube from the armchair to the drawer.\n2. Remove blue cube from the armchair to the drawer.\n3. Remove green cube from the armchair to the drawer.\n",
#         )
#         self._conv.append_message(
#             self._conv.roles[0], "How would you move all toys into the box?\n"
#         )
#         self._conv.append_message(
#             self._conv.roles[1], "List of objects to find: ['toys']\n"
#         )
#         self._conv.append_message(
#             self._conv.roles[0], "List of found objects: ['toy cat', 'toy cube', 'car']\n"
#         )
#         self._conv.append_message(
#             self._conv.roles[1],
#             "1. Move toy cat into the box.\n2. Move toy cube into the box.\n3. Move car into the box.\n",
#         )
#         # # Example red objects
#         self._conv.append_message(
#             self._conv.roles[0],
#             "How would you hide all red objects inside bedside table?\n",
#         )
#         self._conv.append_message(
#             self._conv.roles[1], "List of objects to find: ['red objects']\n"
#         )
#         self._conv.append_message(
#             self._conv.roles[0],
#             "List of found objects: ['red cube', 'red cat', 'red apple']\n",
#         )
#         self._conv.append_message(
#             self._conv.roles[1],
#             "1. Hide red cube inside bedside table.\n2. Hide red cat inside bedside table.\n3. Hide red apple inside bedside table.\n",
#         )
