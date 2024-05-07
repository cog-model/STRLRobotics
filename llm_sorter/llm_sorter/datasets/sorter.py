import json
from dataclasses import dataclass, field
from itertools import chain, product
from pathlib import Path
from typing import Any, List, Optional

from llm_sorter import WandbLogger

from .base import BaseDataset, BaseTask


@dataclass
class SorterStep:
    action: str = ""
    arguments: List[str] = field(default_factory=list)
    text: str = ""
    embedding: Any = field(default_factory=lambda: None, repr=False)


@dataclass
class SorterTask(BaseTask):
    goal: str = ""
    steps: List[SorterStep] = field(default_factory=list)
    text: str = ""
    task_type: int = -1
    plan_id: int = -1

    def __post_init__(self):
        if self.goal.endswith("."):
            self.goal = self.goal[:-1]


class SorterDataset(BaseDataset):
    def __init__(
        self,
        logger: WandbLogger,
        path_to_data_dir: Path = Path("."),
        dataset_filename: Optional[str] = None,
        dataset_ext: str = "json",
    ):
        path_to_data_dir = Path(path_to_data_dir)
        self.path_to_dataset = path_to_data_dir / f"{dataset_filename}.{dataset_ext}"
        super().__init__(logger=logger)

        with open(self.path_to_dataset, "r") as f:
            js = json.load(f)
        self._data = js
        self._size = len(self._data)

        if len(self) == 0:
            raise ValueError("No data")

        self.actions = set()
        self.objects = set()
        self.receptacles = set()

        for item in self:
            for step in item.steps:
                self.actions.add(step.action)
                if len(step.arguments) == 2:
                    self.objects.add(step.arguments[0])
                    self.receptacles.add(step.arguments[1])

        self._logger.info(f"Possible actions:     {self.actions}")
        self._logger.info(f"Possible objects:     {self.objects}")
        self._logger.info(f"Possible receptacles: {self.receptacles}")

        #     for i, step in enumerate(element['plan']):
        #         if step[0] == 'find':
        #             continue
        #         elif step[0] == 'pick_up':
        #             steps.append(['pick_up', step[1][::-1]])
        #         elif step[0] == 'put':
        #             recepticle = element['plan'][i - 1][1][0]
        #             steps.append(['put', step[1] + [recepticle]])
        #         else:
        #             steps.append(step)
        #     element['plan'] = steps

        # for arg_idx, argument in enumerate(step[1:]):
        # pass
        #             if isinstance(argument, list):
        #                 arguments.append(argument[0])
        #             else:
        #                 arguments.append(argument)
        # for element in self._data:
        #     for i, step in enumerate(element['plan']):
        #         output = []
        #         output.append(step[0])
        #         arguments = []
        #         for arg_idx, argument in enumerate(step[1:]):
        #             if isinstance(argument, list):
        #                 arguments.append(argument[0])
        #             else:
        #                 arguments.append(argument)
        #         output.append(arguments)
        #         element['plan'][i] = output
        # with open('out_plan.json' ,'w') as f:
        #     json.dump(self._data, f, ensure_ascii=False)

    def generate_all_possible_steps(self) -> List[SorterStep]:
        possible_steps = []
        for action in self.actions:
            if action == "put" or action == "pick_up":
                for obj, recept in product(self.objects, self.receptacles):
                    possible_steps.append(
                        SorterStep(action=action, arguments=[obj, recept])
                    )
            elif action == "move_to":
                for target in chain(self.objects, self.receptacles):
                    possible_steps.append(SorterStep(action=action, arguments=[target]))
        return possible_steps

    def __len__(self):
        return self._size

    def get_data(self):
        pass

    def __getitem__(self, idx) -> SorterTask:
        plan = self._data[idx]
        steps = []
        for step in plan["plan"]:
            steps.append(SorterStep(action=step[0], arguments=step[1][::-1]))

        return SorterTask(
            goal=plan["goal_eng"],
            steps=steps,
            task_type=plan["task_type"],
            plan_id=plan["plan_id"],
        )
