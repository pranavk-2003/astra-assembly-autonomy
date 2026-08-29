"""Explicit FSM states, exactly the set given in the assessment spec's state diagram:
LOAD_JOB -> VALIDATE -> BUILD_WORLD -> PLAN_SKILL -> EXECUTE_SKILL -> VERIFY ->
NEXT_STEP -> COMPLETE, with REPLAN_RETRY and RECOVERY/ABORT branches."""
from __future__ import annotations

import enum


class State(enum.Enum):
    LOAD_JOB = "LOAD_JOB"
    VALIDATE = "VALIDATE"
    BUILD_WORLD = "BUILD_WORLD"
    PLAN_SKILL = "PLAN_SKILL"
    EXECUTE_SKILL = "EXECUTE_SKILL"
    VERIFY = "VERIFY"
    NEXT_STEP = "NEXT_STEP"
    COMPLETE = "COMPLETE"
    REPLAN_RETRY = "REPLAN_RETRY"
    RECOVERY = "RECOVERY"
    ABORT = "ABORT"


class JobStatus(enum.Enum):
    COMPLETE = "complete"
    ABORTED = "aborted"
    OPERATOR_PAUSED = "operator_paused"
