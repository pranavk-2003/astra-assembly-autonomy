"""R11 test 6: FSM behaviour - a dry-run job reaches COMPLETE; an injected
failure reaches RECOVERY and the trace records the classified failure class
and chosen action; enough repeated failures reaches the policy's terminal
action (ABORT or OPERATOR_PAUSE), deterministically (the assessment spec's recovery
policy must be "deterministic, not incidental")."""
from astra_core.orchestrator.job_runner import run_job
from astra_core.orchestrator.states import JobStatus
from astra_core.ports.execution_port import ExecutionStatus
from astra_core.ports.planner_port import FailureReason
from astra_core.recipe.loader import load_recipe
from astra_core.recovery.policy import POLICY, FailureClass
from astra_core.skills.base import SkillContext
from astra_core.trace.logger import TraceLogger
from astra_sim.fault_injector import FaultInjector
from astra_sim.mock_executor import MockExecutor
from astra_sim.mock_planner import MockPlanner
from astra_sim.mock_scene import MockScene


def _ctx(recipe, fault_injector=None):
    return SkillContext(
        planner=MockPlanner(fault_injector),
        execution=MockExecutor(fault_injector),
        scene=MockScene(),
        logger=TraceLogger(job_id=recipe.job_id),
        job_id=recipe.job_id,
    )


def test_clean_run_reaches_complete():
    recipe = load_recipe("recipes/ASTRA_Pranav_Variant_A.json")
    result = run_job(recipe, _ctx(recipe))
    assert result.status is JobStatus.COMPLETE
    assert result.steps_completed == result.steps_total


def test_single_transient_ik_failure_recovers_via_replan():
    recipe = load_recipe("recipes/ASTRA_Pranav_Variant_A.json")
    injector = FaultInjector.with_plan_failures(FailureReason.IK_UNREACHABLE)
    ctx = _ctx(recipe, injector)
    result = run_job(recipe, ctx)

    assert result.status is JobStatus.COMPLETE  # recovered after one re-plan
    logged = ctx.logger.records
    recovery_events = [r for r in logged if r.get("event") == "fsm_state" and r["state"] == "RECOVERY"]
    assert len(recovery_events) == 1
    assert recovery_events[0]["failure_class"] == FailureClass.IK_UNREACHABLE.value
    assert recovery_events[0]["action"] == "replan"


def test_persistent_ik_failure_reaches_terminal_abort():
    recipe = load_recipe("recipes/ASTRA_Pranav_Variant_A.json")
    n_policy_steps = len(POLICY[FailureClass.IK_UNREACHABLE])
    injector = FaultInjector.with_plan_failures(
        *([FailureReason.IK_UNREACHABLE] * (n_policy_steps + 2))
    )
    ctx = _ctx(recipe, injector)
    result = run_job(recipe, ctx)

    assert result.status is JobStatus.ABORTED
    abort_events = [r for r in ctx.logger.records if r.get("event") == "fsm_state" and r["state"] == "ABORT"]
    assert len(abort_events) == 1
    assert abort_events[0]["final_status"] == "aborted"


def test_execution_error_never_auto_retries_and_pauses_for_operator():
    recipe = load_recipe("recipes/ASTRA_Pranav_Variant_A.json")
    injector = FaultInjector.with_exec_failures(ExecutionStatus.PROTECTIVE_STOP)
    ctx = _ctx(recipe, injector)
    result = run_job(recipe, ctx)

    assert result.status is JobStatus.OPERATOR_PAUSED
    # exactly one plan/execute attempt total - "no auto-retry" means the step
    # is never dispatched a second time after the controller error.
    assert len(ctx.execution.executed) == 1
    recovery_events = [r for r in ctx.logger.records if r.get("event") == "fsm_state" and r["state"] == "RECOVERY"]
    assert recovery_events[0]["action"] == "operator_pause"


def test_pick_verify_failure_retries_then_recovers():
    recipe = load_recipe("recipes/ASTRA_Pranav_Variant_A.json")
    injector = FaultInjector(pick_verify_failures=1)
    ctx = _ctx(recipe, injector)
    result = run_job(recipe, ctx)

    assert result.status is JobStatus.COMPLETE
    recovery_events = [r for r in ctx.logger.records if r.get("event") == "fsm_state" and r["state"] == "RECOVERY"]
    assert recovery_events[0]["failure_class"] == FailureClass.PICK_VERIFY_FAILED.value


def test_perception_rejection_pauses_before_any_step_executes():
    from astra_core.recipe.loader import load_correction

    recipe = load_recipe("recipes/ASTRA_Pranav_Variant_B.json")
    failure = load_correction("recipes/ASTRA_Pranav_Failure_Injection.json")
    ctx = _ctx(recipe)
    result = run_job(recipe, ctx, correction=failure)

    assert result.status is JobStatus.OPERATOR_PAUSED
    assert result.steps_completed == 0
    gate_events = [r for r in ctx.logger.records if r.get("event") == "perception_gate"]
    assert gate_events[0]["gate_accepted"] is False
