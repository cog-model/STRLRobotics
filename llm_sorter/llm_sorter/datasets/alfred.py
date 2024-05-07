from dataclasses import dataclass, field
from enum import Enum
import json
from pathlib import Path
from typing import Dict, List, Optional
from llm_sorter.datasets import BaseTask
from llm_sorter.datasets import BaseDataset
from llm_sorter import WandbLogger


class AlfredTaskType(Enum):
    """Task types from ALFRED dataset"""

    look_at_obj_in_light = "look at object in light"
    pick_and_place_simple = "pick and place"
    pick_two_obj_and_place = "pick two objects and place"
    pick_and_place_with_movable_recep = "pick and place with movable recepticle"
    pick_heat_then_place_in_recep = "pick and heat then place in recepticle"
    pick_cool_then_place_in_recep = "pick and cool then place in recepticle"
    pick_clean_then_place_in_recep = "pick and clean then place in recepticle"
    not_set = "not set"


class AlfredActions(Enum):
    """Actions from ALFRED dataset"""

    PickupObject = "pick_up"
    ToggleObject = "toggle"
    PutObject = "put"
    SliceObject = "slice"
    CleanObject = "clean"
    HeatObject = "heat"
    CoolObject = "cool"


@dataclass
class AlfredStep:
    """Step from ALFRED dataset"""

    action: str = ""
    arguments: List[str] = field(default_factory=list)
    text: str = ""


@dataclass
class AlfredTask(BaseTask):
    """Task from ALFRED dataset"""

    goal: str = ""
    steps: List[AlfredStep] = field(default_factory=list)
    text: str = ""
    task_type: AlfredTaskType = AlfredTaskType.not_set
    plan_id: int = -1


class AlfredDataset(BaseDataset):
    """ALFRED dataset"""

    _data: List[Dict]
    _size: int

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

    def __len__(self):
        return self._size

    def __getitem__(self, idx) -> AlfredTask:
        entry: Dict = self._data[idx]

        steps: AlfredStep = []
        for step in entry["list_of_actions"]:
            # step = ['object', 'Action']
            alfred_step = AlfredStep(
                action=AlfredActions[step[1]].value, arguments=[step[0]]
            )

            steps.append(alfred_step)

        task = AlfredTask(
            goal=entry["goal"],
            task_type=AlfredTaskType[entry["task_type"]],
            plan_id=idx,
            steps=steps,
        )

        return task


if __name__ == "__main__":
    logger = WandbLogger()
    dataset = AlfredDataset(
        logger,
        path_to_data_dir="/home/patratskiy_ma/work/llm_sorter_sber/llm_sorter/data/alfred/",
        dataset_filename="valid_unseen_highlevel",
    )

    item = dataset[0]
    print(item)
    print("Done")
