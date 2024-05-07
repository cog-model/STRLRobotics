from abc import ABC
from dataclasses import dataclass, field
from enum import Enum
from typing import List

from llm_sorter import WandbLogger


class EventType(Enum):
    ROBOT_ACTION = 1
    ITEMS = 2


@dataclass
class BaseEvent:
    type: EventType


@dataclass
class Item:
    name: str

    def __repr__(self) -> str:
        return self.name


@dataclass
class ItemsEvent:
    type: EventType = EventType.ITEMS
    items: List[Item] = field(default_factory=list)


class BaseEnv(ABC):
    def __init__(self, logger: WandbLogger, **kwargs) -> None:
        self._logger = logger
        self._events_list: List[BaseEvent] = []
