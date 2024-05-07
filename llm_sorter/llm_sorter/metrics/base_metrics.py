from abc import ABC, abstractmethod
from typing import Any, Callable, List, Optional

from llm_sorter import BaseLogger
from llm_sorter.datasets import BaseTask
from llm_sorter.processors import BaseProcessor


def preprocess(func):
    """Preprocess pred and target with given function before calculating metrics."""

    def wrapper(self, pred: BaseTask, target: BaseTask):
        if self._pred_process_f is not None:
            pred = self._pred_process_f(pred)
        if self._target_process_f is not None:
            target = self._target_process_f(target)
        return func(self, pred, target)

    return wrapper


class BaseMetric(ABC):
    """Base class for calculating metrics"""

    _name: str

    @property
    def name(self):
        return self._name

    @abstractmethod
    def __init__(
        self,
        pred_process_f: Optional[Callable] = None,
        target_process_f: Optional[Callable] = None,
        name: str = "",
        **kwargs,
    ):
        self._pred_process_f = pred_process_f
        self._target_process_f = pred_process_f
        self._name = name

    @abstractmethod
    def __call__(self, pred: Any, target: Any) -> float:
        raise NotImplementedError


class BaseTaskMetrics(ABC):
    """Base class for set of metrics calculated specially for dataset"""

    def __init__(self, logger: BaseLogger, processor: BaseProcessor, **kwargs):
        self._metric_list: List[BaseMetric] = []
        self._processor = processor
        self._logger = logger

    @abstractmethod
    def update(self, predicted_task: BaseTask, target_task: BaseTask) -> Any:
        """Calculate the intermediate results and store them in class."""
        raise NotImplementedError

    @abstractmethod
    def calculate_metrics(self) -> Any:
        """Calculate total results for the entire dataset."""
        raise NotImplementedError
