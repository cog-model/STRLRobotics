from dataclasses import dataclass
from enum import auto, Enum
from typing import List
import json


class Acts(Enum):
    move_to = auto()
    pick_up = auto()
    put = auto()


class Items(Enum):
    toy_cat = auto()
    cat = auto()
    stuffed_cat = auto()
    orange_cat = auto()
    box = auto()
    toy_box = auto()
    red_box = auto()
    orange_box = auto()


class Places(Enum):
    unspecified = auto
    table = auto()
    drawer = auto()
    box = auto()
    container = auto()
    white_container = auto()
    orange_container = auto()
    red_container = auto()
    floor = auto()


@dataclass
class Step:
    act: Acts
    item: Items
    place: Places

    def to_json(self) -> List:
        return [
            self.act.name.lower(),
            [
                self.place.name.lower().replace("_", " "),
                self.item.name.lower().replace("_", " "),
            ],
        ]


@dataclass
class Task:
    goal_rus: str
    goal_eng: str
    steps: List[Step]

    def to_json(self, id: int = 0):
        out = {}
        out["task_type"] = "test"
        out["plan_id"] = id
        out["goal_eng"] = self.goal_eng
        out["goal_rus"] = self.goal_rus

        plan = []
        for step in self.steps:
            plan.append(step.to_json())
        out["plan"] = plan

        return out


tests: List[Task] = [
    Task(
        goal_rus="возьми кота",
        goal_eng="take the cat",
        steps=[
            Step(act=Acts.move_to, item=Items.cat, place=Places.unspecified),
            Step(act=Acts.pick_up, item=Items.cat, place=Places.unspecified),
        ],
    ),
    Task(
        goal_rus="робот возьми кота",
        goal_eng="robot take the cat",
        steps=[
            Step(act=Acts.move_to, item=Items.cat, place=Places.unspecified),
            Step(act=Acts.pick_up, item=Items.cat, place=Places.unspecified),
        ],
    ),
    Task(
        goal_rus="возьми оранжевого кота",
        goal_eng="take the orange cat",
        steps=[
            Step(act=Acts.move_to, item=Items.orange_cat, place=Places.unspecified),
            Step(act=Acts.pick_up, item=Items.orange_cat, place=Places.unspecified),
        ],
    ),
    Task(
        goal_rus="возьми оранжевого кота со стола",
        goal_eng="take the orange cat from the table",
        steps=[
            Step(act=Acts.move_to, item=Items.orange_cat, place=Places.table),
            Step(act=Acts.pick_up, item=Items.orange_cat, place=Places.table),
        ],
    ),
    Task(
        goal_rus="возьми кота со стола",
        goal_eng="take the cat from the table",
        steps=[
            Step(act=Acts.move_to, item=Items.cat, place=Places.table),
            Step(act=Acts.pick_up, item=Items.cat, place=Places.table),
        ],
    ),
    Task(
        goal_rus="робот возьми кота со стола",
        goal_eng="robot take the cat from the table",
        steps=[
            Step(act=Acts.move_to, item=Items.cat, place=Places.table),
            Step(act=Acts.pick_up, item=Items.cat, place=Places.table),
        ],
    ),
    # Task(
    #     goal_rus="подъедь к коту",
    #     goal_eng="drive up to the cat",
    #     steps=[
    #         Step(act=Acts.move_to, item=Items.cat, place=Places.unspecified),
    #     ],
    # ),
    # Task(
    #     goal_rus="подъедь к коту на столе",
    #     goal_eng="drive up to the cat on the table",
    #     steps=[
    #         Step(act=Acts.move_to, item=Items.cat, place=Places.table),
    #     ],
    # ),
    Task(
        goal_rus="доехай до кота",
        goal_eng="get to the cat",
        steps=[
            Step(act=Acts.move_to, item=Items.cat, place=Places.unspecified),
        ],
    ),
    Task(
        goal_rus="доехай до кота на столе",
        goal_eng="get to the cat on the table",
        steps=[
            Step(act=Acts.move_to, item=Items.cat, place=Places.table),
        ],
    ),
    Task(
        goal_rus="Возьми кота и положи его на стол",
        goal_eng="Take the cat and put it on the table",
        steps=[
            Step(act=Acts.move_to, item=Items.cat, place=Places.unspecified),
            Step(act=Acts.pick_up, item=Items.cat, place=Places.unspecified),
            Step(act=Acts.move_to, item=Items.cat, place=Places.table),
            Step(act=Acts.put, item=Items.cat, place=Places.table),
        ],
    ),
    Task(
        goal_rus="Возьми оранжевого кота и положи его на стол",
        goal_eng="Take the orange cat and put it on the table",
        steps=[
            Step(act=Acts.move_to, item=Items.orange_cat, place=Places.unspecified),
            Step(act=Acts.pick_up, item=Items.orange_cat, place=Places.unspecified),
            Step(act=Acts.move_to, item=Items.orange_cat, place=Places.table),
            Step(act=Acts.put, item=Items.orange_cat, place=Places.table),
        ],
    ),
    Task(
        goal_rus="Возьми оранжевого кота c пола и положи его на стол",
        goal_eng="Take the cat off the floor and put it on the table",
        steps=[
            Step(act=Acts.move_to, item=Items.orange_cat, place=Places.floor),
            Step(act=Acts.pick_up, item=Items.orange_cat, place=Places.floor),
            Step(act=Acts.move_to, item=Items.orange_cat, place=Places.table),
            Step(act=Acts.put, item=Items.orange_cat, place=Places.table),
        ],
    ),
]

dataset = []
for i, task in enumerate(tests):
    dataset.append(task.to_json(i))
    print(task.goal_rus)

# pprint(dataset)

with open("./data/llp/llp_tests.json", "w", encoding="utf8") as f:
    json.dump(dataset, f, indent=4, ensure_ascii=False)
