import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional

from llm_sorter import WandbLogger
from llm_sorter.datasets import BaseDataset, BaseTask


@dataclass
class LLPStep:
    """Step in Low Level Planning task"""

    action: str = ""
    arguments: List[str] = field(default_factory=list)
    text: str = ""

    def __str__(self) -> str:
        return f"{self.action}({', '.join(self.arguments)})"

    def __repr__(self) -> str:
        return str(self)


@dataclass
class LLPTask(BaseTask):
    """Low Level Planning task"""

    goal: str = ""
    steps: List[LLPStep] = field(default_factory=list)
    text: str = ""
    task_type: int = -1
    plan_id: int = -1

    def __post_init__(self):
        if self.goal.endswith("."):
            self.goal = self.goal[:-1]

    def __str__(self) -> str:
        return f"{self.goal}"

    def __repr__(self) -> str:
        return str(self)


class LLPDataset(BaseDataset):
    """Low Level Planning dataset"""

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

    def __len__(self):
        return self._size

    def get_data(self):
        pass

    def __getitem__(self, idx) -> LLPTask:
        plan = self._data[idx]
        steps = []
        for step in plan["plan"]:
            steps.append(LLPStep(action=step[0], arguments=step[1][::-1]))

        return LLPTask(
            goal=plan["goal_eng"],
            steps=steps,
            task_type=plan["task_type"],
            plan_id=plan["plan_id"],
        )
