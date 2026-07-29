"""Policy implementations — pure Python, no ROS dependency.

These are the pluggable decision-making algorithms behind the PolicyNode.
They run in either conda Python 3.9 or system Python 3.12.
"""

from abc import ABC, abstractmethod
from typing import Any

from sim_bridge.policy.unified_action import UnifiedAction


class PolicyImpl(ABC):
    """Subclass implements decide() — the decision logic per step."""

    @abstractmethod
    def decide(self, rgb: Any, depth: Any, odom: Any, state: int) -> UnifiedAction:
        """Return next action. Called each step."""
        ...

    def reset(self):
        """Called at episode start."""
        pass

    def close(self):
        """Clean up resources."""
        pass


class RandomExplorerPolicy(PolicyImpl):
    """Random discrete actions, biased toward forward."""

    def decide(self, rgb, depth, odom, state):
        import random
        action_id = random.choice([1, 1, 1, 2, 3])
        return UnifiedAction.from_discrete(action_id)


class ApexNavBridgePolicy(PolicyImpl):
    """Polls ApexNav C++ FSM via the ros2_http_bridge.

    Only works in conda Python 3.9 (needs HabitatEvalBridgeClient which
    imports from habitat2ros). Falls back to STOP if import fails.
    """

    def __init__(self, bridge_url="http://127.0.0.1:18081"):
        self._bridge = None
        try:
            from habitat2ros.ros2_bridge_client import HabitatEvalBridgeClient
            self._bridge = HabitatEvalBridgeClient(
                bridge_url=bridge_url, auto_start=False
            )
            self._bridge.ros_state = 0
        except ImportError:
            pass  # will return STOP on every decide()

    def decide(self, rgb, depth, odom, state):
        if self._bridge is None:
            return UnifiedAction(stop=True)
        self._bridge.spin_once(timeout_sec=0.1)
        action_id = self._bridge.global_action
        self._bridge.global_action = None
        if action_id is not None:
            return UnifiedAction.from_discrete(int(action_id))
        return UnifiedAction(stop=True)

    def reset(self):
        if self._bridge:
            self._bridge.global_action = None

    def close(self):
        if self._bridge:
            self._bridge.close()


class MobileVLAPolicy(PolicyImpl):
    """MobileVLA-R1 — end-to-end VLA navigation.

    TODO: implement when inference code from GitHub is ready.
    Model checkpoint at: model_cache/MobileVLA-R1/weight/rl/
    """

    def __init__(self, model_path="", device="cuda"):
        self._model_path = model_path
        self._device = device
        self._model = None

    def decide(self, rgb, depth, odom, state):
        if self._model is None:
            return UnifiedAction(stop=True)
        # TODO: run real VLA inference
        return UnifiedAction(stop=True)


# ── Registry ───────────────────────────────────────────────────────────

_POLICY_REGISTRY = {
    "random_explorer": RandomExplorerPolicy,
    "apexnav": ApexNavBridgePolicy,
    "mobilevla": MobileVLAPolicy,
}


def get_policy_impl(name: str, **kwargs) -> PolicyImpl:
    """Factory: create a policy implementation by name.

    Extra kwargs are silently ignored if the policy class doesn't accept them.
    """
    if name not in _POLICY_REGISTRY:
        raise KeyError(
            f"Unknown policy: '{name}'. Available: {list(_POLICY_REGISTRY)}"
        )
    cls = _POLICY_REGISTRY[name]
    # Filter kwargs to only what the class __init__ accepts
    import inspect
    sig = inspect.signature(cls.__init__)
    accepted = {k: v for k, v in kwargs.items() if k in sig.parameters}
    return cls(**accepted)
