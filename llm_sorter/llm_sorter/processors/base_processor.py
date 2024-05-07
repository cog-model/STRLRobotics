from abc import ABC, abstractmethod

from llm_sorter import WandbLogger
from llm_sorter.datasets import BaseTask
from llm_sorter.models import BaseModelInput, BaseModelOutput


class BaseProcessor(ABC):
    def __init__(self, logger: WandbLogger, **kwargs):
        self._logger = logger

    @abstractmethod
    def to_inputs(self, task: BaseTask) -> BaseModelInput:
        raise NotImplementedError

    @abstractmethod
    def to_task(self, task: BaseModelOutput) -> BaseTask:
        raise NotImplementedError


class ProcessorWithSteps(BaseProcessor):
    def __init__(self, logger: WandbLogger, **kwargs):
        super().__init__(logger, **kwargs)

    @abstractmethod
    def get_next_step(self, BaseOutput, **kwargs):
        raise NotImplementedError
