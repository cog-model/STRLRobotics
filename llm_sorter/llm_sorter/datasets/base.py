from abc import ABC, abstractmethod
from dataclasses import dataclass

from llm_sorter import BaseLogger


@dataclass
class BaseTask(ABC):
    """Base class for Task"""

    pass


class BaseDataset(ABC):
    """Base class for dataset"""

    def __init__(self, logger: BaseLogger) -> None:
        self._logger = logger

    @abstractmethod
    def __len__(self):
        raise NotImplementedError

    @abstractmethod
    def __getitem__(self, idx) -> BaseTask:
        raise NotImplementedError
