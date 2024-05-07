from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List, Optional
import torch


@dataclass
class BaseInput(ABC):
    """Base class for input to LLM models"""

    text: Optional[str] = None


@dataclass
class ScoringInput(BaseInput):
    """Input to LLM models for scoring"""

    text: Optional[str] = None
    options: Optional[List[str]] = None


@dataclass
class BaseOutput(ABC):
    """Base class for output from LLM models"""

    text: Optional[str] = None


@dataclass
class ScoringOutput(ABC):
    """Output from LLM models for scoring"""

    scores: Optional[torch.Tensor] = None


class BaseLLMModel(ABC):
    """Base class for LLM models"""

    _name: str

    @property
    def name(self):
        return self._name

    def __init__(self, name: str = "Base Model"):
        self._name = name

    @abstractmethod
    def _load(self) -> None:
        """Load model, tokenizer, etc"""

    @abstractmethod
    def generate(self, prompt: str) -> str:
        """Generate text"""

    @abstractmethod
    def score(self, option: str) -> float:
        """Score one option"""
