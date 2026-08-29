"""Thin state-transition logger: every state entry from the mandated set
(LOAD_JOB -> ... -> COMPLETE / REPLAN_RETRY / RECOVERY / ABORT, per the assessment spec)
is written to the trace log, so a reviewer can reconstruct the FSM's path for
any run from logs alone (R10)."""
from __future__ import annotations

from astra_core.orchestrator.states import State
from astra_core.trace.logger import TraceLogger


def enter_state(logger: TraceLogger, state: State, **extra) -> None:
    logger.log(event="fsm_state", state=state.value, **extra)
