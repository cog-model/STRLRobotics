from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List, Optional

from llm_sorter import BaseLogger


@dataclass
class BaseModelInput(ABC):
    text: Optional[str] = None


@dataclass
class BaseModelOutput(ABC):
    text: Optional[str] = None


@dataclass
class ScoringInput(BaseModelInput):
    text: Optional[str] = None
    options: Optional[List[str]] = None


@dataclass
class ScoringOutput(ABC):
    scores: Optional[List] = None


class BaseLLMModel(ABC):
    """Base class for LLM models"""

    _name: str
    _logger: BaseLogger

    @property
    def name(self):
        return self._name

    def __init__(self, name: str, logger: BaseLogger, **kwargs) -> None:
        self._name = name

        self._logger = logger
        self._logger.info(f"Model: {name}")

    @abstractmethod
    def generate(self, inputs: BaseModelInput, **kwargs) -> BaseModelOutput:
        """Generate text"""
        raise NotImplementedError

    @abstractmethod
    def score_text(self, inputs: ScoringInput, **kwargs) -> ScoringOutput:
        """Score text for saycan approach"""
        raise NotImplementedError
