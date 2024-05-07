from dataclasses import dataclass, field
import json
from pathlib import Path
from typing import List, Optional
from llm_sorter.datasets import BaseTask, LLPTask
from llm_sorter.datasets import BaseDataset
from llm_sorter.environment import ItemsEvent
from llm_sorter import WandbLogger


@dataclass
class CleanRoomTask(BaseTask):
    """Task for Clean Room scenario"""

    goal: str = ""
    clean_room: bool = False
    task_type: str = "none"
    plan_id: int = -1


@dataclass
class VCTask(BaseTask):
    """Task for Valid Check scenario"""

    goal: str = ""
    valid_plan: bool = False
    task_type: str = "none"
    plan_id: int = -1


@dataclass
class HLPTask(BaseTask):
    """Task for High Level Planning scenario"""

    goal: str = ""
    subtasks: List[LLPTask] = field(default_factory=list)
    feedback: ItemsEvent = field(default_factory=ItemsEvent)
    request: str = ""
    text: str = ""
    task_type: str = "none"
    plan_id: int = -1
    valid_plan: bool = False
    clean_room: bool = False

    def __post_init__(self):
        if self.goal.endswith("."):
            self.goal = self.goal[:-1]


@dataclass
class SpecTask(BaseTask):
    """Task for Specification scenario"""

    goal: str = ""
    request: str = ""
    task_type: str = "none"
    plan_id: int = -1

    def __post_init__(self):
        if self.goal.endswith("."):
            self.goal = self.goal[:-1]

    @property
    def need_feedback(self) -> bool:
        return self.request.lower() != "none"


class HLPDataset(BaseDataset):
    """High Level Planning dataset"""

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

    def get_data(self):
        pass

    def __getitem__(self, idx) -> HLPTask:
        plan = self._data[idx]
        llp_tasks = []
        for llp_goal in plan["plan"]:
            llp_tasks.append(LLPTask(goal=llp_goal))

        request = plan["request"]
        if request is None:
            request = "none"

        clean_room = plan.get("clean_room", False)

        return HLPTask(
            goal=plan["goal_eng"],
            subtasks=llp_tasks,
            feedback=ItemsEvent(items=plan["items"]),
            task_type=plan["task_type"],
            plan_id=plan["plan_id"],
            valid_plan=plan["valid_plan"],
            clean_room=clean_room,
            request=request,
        )
