from geometry_msgs.msg import PoseStamped, Twist
from luxinav_interfaces.msg import Decision, RunContext


def test_generated_decision_union_and_identity_are_usable():
    context = RunContext(
        contract_version="luxinav.v1",
        run_id="run-1",
        episode_id="mock-000",
        frame_id=7,
        source_plugin_id="mock",
    )
    decision = Decision(context=context)

    assert decision.context.run_id == "run-1"
    assert decision.CONTINUOUS_CONTROL == 1
    assert decision.DISCRETE_ACTION == 2
    assert decision.NAVIGATION_GOAL == 3
    assert decision.STOP == 4
    assert isinstance(decision.continuous_control, Twist)
    assert isinstance(decision.navigation_goal, PoseStamped)
