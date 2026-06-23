"""URAF plugin base interfaces (PLAN.md §20)."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any


class UrafPlugin(ABC):
    """Base class for all URAF plugins."""

    plugin_type: str = "generic"

    @abstractmethod
    def initialize(self, config: dict[str, Any]) -> bool:
        ...

    @abstractmethod
    def status(self) -> dict[str, Any]:
        ...


class FALDriverPlugin(UrafPlugin):
    plugin_type = "fal_driver"

    @abstractmethod
    def connect(self, config: dict[str, Any]) -> bool:
        ...

    @abstractmethod
    def write_command(self, joint_id: int, value: float) -> bool:
        ...


class PerceptionPlugin(UrafPlugin):
    plugin_type = "perception"

    @abstractmethod
    def detect(self, image_path: str) -> list[dict]:
        ...


class TaskPrimitivePlugin(UrafPlugin):
    plugin_type = "task_primitive"

    @abstractmethod
    def execute(self, context: dict[str, Any]) -> bool:
        ...
