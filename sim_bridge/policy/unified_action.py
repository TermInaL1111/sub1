"""UnifiedAction — canonical action representation for all policies and simulators.

Continuous velocity parameters are based on habitat_physics_task.py
_set_agent_velocities(), which converts Habitat discrete actions (0-5) to
continuous linear/angular velocity with control_period=1.0s:

    MOVE_FORWARD → linear_velocity  = 0.25 m/s     (Habitat local -Z = forward)
    TURN_LEFT    → angular_velocity = +10°/s       (Habitat local +Y = yaw)
    TURN_RIGHT   → angular_velocity = -10°/s
    STOP         → all zeros

Reference:
    ros_x_habitat-master/src/tasks/habitat_physics_task.py L149-194
"""

import math
from dataclasses import dataclass


# ---------------------------------------------------------------------------
# Discrete action IDs (matches ApexNav params.py)
# ---------------------------------------------------------------------------

class ACTION:
    STOP = 0
    MOVE_FORWARD = 1
    TURN_LEFT = 2
    TURN_RIGHT = 3
    TURN_DOWN = 4
    TURN_UP = 5

    NAMES = {
        0: "STOP",
        1: "MOVE_FORWARD",
        2: "TURN_LEFT",
        3: "TURN_RIGHT",
        4: "TURN_DOWN",
        5: "TURN_UP",
    }

    # ── Continuous velocity parameters (from habitat_physics_task.py) ─
    # With control_period=1.0s:
    #   FORWARD_M:  0.25 m / 1.0s = 0.25 m/s  (Habitat _C.SIMULATOR.FORWARD_STEP_SIZE)
    #   TURN_DEG:   10°  / 1.0s = 10°/s       (Habitat _C.SIMULATOR.TURN_ANGLE)
    FORWARD_M = 0.25
    TURN_DEG = 10.0
    TURN_RAD_PER_S = math.radians(TURN_DEG)     # 0.1745… rad/s

    # Camera tilt is instantaneous (not velocity-based), matched to
    # HabitatWrapper.TILT_ANGLE = π/6 (30° per action)
    TILT_RAD = math.pi / 6.0

    # Teleport-mode displacement (for step_duration calculation)
    # In Habitat teleport mode one discrete step turns 30°, not 10°.
    # This is the TARGET displacement that velocity commands aim to match.
    TELEPORT_TURN_RAD = math.pi / 6.0


# ---------------------------------------------------------------------------
# UnifiedAction
# ---------------------------------------------------------------------------

@dataclass
class UnifiedAction:
    """
    Canonical action shared by ALL policies and ALL simulators.

    Fields (all velocities in ROS convention: x=forward, z=yaw):
        vx:       linear velocity x (m/s), forward
        vy:       linear velocity y (m/s), lateral
        yaw_rate: angular velocity around z (rad/s)
        pitch:    camera pitch delta (rad), +=look up
        stop:     stop signal (overrides all other fields)
    """

    vx: float = 0.0
    vy: float = 0.0
    yaw_rate: float = 0.0
    pitch: float = 0.0
    stop: bool = False

    # ── Discrete → Unified (velocity-based, matches habitat_physics_task) ─

    @classmethod
    def from_discrete(cls, action_id: int) -> "UnifiedAction":
        """Convert ApexNav discrete action (0-5) → UnifiedAction.

        Velocity values match habitat_physics_task.py _set_agent_velocities():
          - MOVE_FORWARD → vx = 0.25 m/s
          - TURN_LEFT    → yaw_rate = +10°/s (0.1745 rad/s)
          - TURN_RIGHT   → yaw_rate = -10°/s
          - TURN_DOWN    → pitch = -30°   (instant tilt, not velocity)
          - TURN_UP      → pitch = +30°
          - STOP         → stop=True
        """
        return _DISCRETE_TO_UNIFIED.get(action_id, cls(stop=True))

    # ── Unified → Discrete (threshold-based) ──────────────────────────────

    def to_discrete(self) -> int:
        """
        Convert continuous velocities → nearest discrete action (0-5).

        Priority: stop > pitch > yaw > linear.
        Thresholds are 50% of the physics_task velocity values so that
        any significant signal maps to the expected action.
        """
        if self.stop:
            return ACTION.STOP

        # Camera tilt (instantaneous, not velocity — so threshold is 1°)
        if self.pitch < -0.017:
            return ACTION.TURN_DOWN
        if self.pitch > 0.017:
            return ACTION.TURN_UP

        # Turn: threshold = 5°/s (half of the 10°/s command)
        _HALF_TURN = math.radians(ACTION.TURN_DEG / 2.0)  # 0.087… rad/s
        if self.yaw_rate > _HALF_TURN:
            return ACTION.TURN_LEFT
        if self.yaw_rate < -_HALF_TURN:
            return ACTION.TURN_RIGHT

        # Forward/lateral motion → MOVE_FORWARD (any significant linear vel)
        if abs(self.vx) > 0.01 or abs(self.vy) > 0.01:
            return ACTION.MOVE_FORWARD

        return ACTION.STOP

    # ── Unified → ROS ─────────────────────────────────────────────────────

    def to_twist(self):
        """Convert → geometry_msgs/Twist (for Isaac, Gazebo, trajectory mode)."""
        from geometry_msgs.msg import Twist

        t = Twist()
        t.linear.x = self.vx
        t.linear.y = self.vy
        t.angular.z = self.yaw_rate
        return t


# ── Discrete → Unified lookup table ─────────────────────────────────────
# Velocity values from habitat_physics_task.py _set_agent_velocities()
# with control_period=1.0s.

_DISCRETE_TO_UNIFIED: dict[int, UnifiedAction] = {
    0: UnifiedAction(stop=True),
    1: UnifiedAction(vx=ACTION.FORWARD_M),                        # 0.25 m/s
    2: UnifiedAction(yaw_rate=ACTION.TURN_RAD_PER_S),             # +10°/s
    3: UnifiedAction(yaw_rate=-ACTION.TURN_RAD_PER_S),            # -10°/s
    4: UnifiedAction(pitch=-ACTION.TILT_RAD),                     # -30° tilt
    5: UnifiedAction(pitch=ACTION.TILT_RAD),                      # +30° tilt
}


# ── Step duration helper ─────────────────────────────────────────────────

def step_duration(vx: float = 0.0, yaw_rate: float = 0.0) -> float:
    """
    How long (seconds) to hold a continuous velocity command to match
    the same physical displacement as one Habitat TELEPORT step.

    Teleport displacements:
        MOVE_FORWARD → 0.25 m
        TURN          → 30° (π/6 rad)

    Returns 0.0 if no motion (STOP) or pitch-only (camera tilt).
    """
    if abs(yaw_rate) > 0.01:
        return ACTION.TELEPORT_TURN_RAD / abs(yaw_rate)
    if abs(vx) > 0.01:
        return ACTION.FORWARD_M / abs(vx)
    return 0.0
