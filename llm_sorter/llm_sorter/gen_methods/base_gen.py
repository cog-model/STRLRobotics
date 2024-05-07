from abc import ABC, abstractmethod
from typing import Any

from llm_sorter import BaseLogger
from llm_sorter.datasets import BaseTask
from llm_sorter.models import BaseLLMModel
from llm_sorter.processors import BaseProcessor


class BasePlanGeneration(ABC):
    """Base class for plan generation methods."""

    def __init__(self, model: BaseLLMModel, logger: BaseLogger, **kwargs):
        self._model = model
        self._logger = logger

    @abstractmethod
    def predict(self, inputs: BaseTask, processor: BaseProcessor, **kwargs) -> Any:
        """Predict complete plan"""
        raise NotImplementedError
