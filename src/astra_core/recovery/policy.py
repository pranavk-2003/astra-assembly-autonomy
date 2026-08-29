"""Deterministic per-failure-class recovery policy (the assessment spec's failure-classes
policy requirement). Each class maps to a FIXED, ordered action sequence
- no randomness, no ad-hoc branching. The orchestrator walks the sequence by
attempt index and clamps at the last (terminal) action."""
from __future__ import annotations

import enum


class FailureClass(enum.Enum):
    IK_UNREACHABLE = "ik_unreachable"
    PLANNING_COLLISION = "planning_collision"
    PERCEPTION_REJECTED = "perception_rejected"
    EXECUTION_ERROR = "execution_error"
    PICK_VERIFY_FAILED = "pick_verify_failed"
    COMM_TIMEOUT = "comm_timeout"


class RecoveryAction(enum.Enum):
    RETRY = "retry"
    REPLAN = "replan"
    RE_OBSERVE = "re_observe"
    SAFE_POSE = "safe_pose"
    OPERATOR_PAUSE = "operator_pause"
    ABORT = "abort"


# Ordered action sequence per failure class. The orchestrator retries the same
# skill/step through each entry in order as failures repeat; the final entry
# is terminal for that failure class.
POLICY: dict[FailureClass, tuple[RecoveryAction, ...]] = {
    FailureClass.IK_UNREACHABLE: (
        RecoveryAction.REPLAN,
        RecoveryAction.REPLAN,
        RecoveryAction.SAFE_POSE,
        RecoveryAction.ABORT,
    ),
    FailureClass.PLANNING_COLLISION: (
        RecoveryAction.REPLAN,
        RecoveryAction.REPLAN,
        RecoveryAction.SAFE_POSE,
        RecoveryAction.OPERATOR_PAUSE,
    ),
    FailureClass.PERCEPTION_REJECTED: (
        RecoveryAction.RE_OBSERVE,
        RecoveryAction.RE_OBSERVE,
        RecoveryAction.OPERATOR_PAUSE,
    ),
    # "No auto-retry" (the assessment spec) means the step must not be attempted again
    # after a controller error/protective stop: moving to a safe pose is
    # bundled into the single, immediate pause transition below rather than
    # spent as a separate retry cycle - unlike IK/collision failures, which
    # do get bounded re-plan attempts before their own safe-pose+terminal step.
    FailureClass.EXECUTION_ERROR: (
        RecoveryAction.OPERATOR_PAUSE,
    ),
    FailureClass.PICK_VERIFY_FAILED: (
        RecoveryAction.RE_OBSERVE,
        RecoveryAction.RETRY,
        RecoveryAction.RETRY,
        RecoveryAction.ABORT,
    ),
    FailureClass.COMM_TIMEOUT: (
        RecoveryAction.RETRY,
        RecoveryAction.RETRY,
        RecoveryAction.RETRY,
        RecoveryAction.ABORT,
    ),
}


def next_action(failure_class: FailureClass, attempt_index: int) -> RecoveryAction:
    """attempt_index is 0-based count of prior failures for this failure class
    within the current step. Clamps to the policy's terminal action."""
    sequence = POLICY[failure_class]
    return sequence[min(attempt_index, len(sequence) - 1)]


def is_terminal(action: RecoveryAction) -> bool:
    return action in (RecoveryAction.ABORT, RecoveryAction.OPERATOR_PAUSE)
