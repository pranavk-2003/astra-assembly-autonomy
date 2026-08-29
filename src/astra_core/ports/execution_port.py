"""ExecutionPort: simulation today, shaped for a real Doosan/UR/ABB/FANUC
controller tomorrow (R12) - trajectory handoff, feedback, protective stop."""
from __future__ import annotations

import abc
import enum
from dataclasses import dataclass


class ExecutionStatus(enum.Enum):
    SUCCESS = "success"
    CONTROLLER_ERROR = "controller_error"
    PROTECTIVE_STOP = "protective_stop"
    COMM_TIMEOUT = "comm_timeout"


@dataclass(frozen=True)
class ExecutionResult:
    status: ExecutionStatus

    @property
    def ok(self) -> bool:
        return self.status is ExecutionStatus.SUCCESS


class ExecutionPort(abc.ABC):
    @abc.abstractmethod
    def execute(self, trajectory: object) -> ExecutionResult: ...

    @abc.abstractmethod
    def attach(self, object_id: str) -> None: ...

    @abc.abstractmethod
    def detach(self, object_id: str) -> None: ...

    @abc.abstractmethod
    def verify_grasp(self, object_id: str) -> bool: ...

    @abc.abstractmethod
    def stop(self) -> None: ...
