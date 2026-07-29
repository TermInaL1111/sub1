"""Conversions between simulator-neutral backend values and ROS messages."""

from __future__ import annotations

import math

from builtin_interfaces.msg import Time
from geometry_msgs.msg import PointStamped, PoseStamped
from luxinav_interfaces.msg import Decision, RunContext
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .backend_protocol import DecisionPayload, ImagePayload, Observation


def run_context(
    observation: Observation,
    stamp: Time,
    *,
    contract_version: str,
    source_plugin_id: str,
) -> RunContext:
    return RunContext(
        contract_version=contract_version,
        run_id=observation.run_id,
        episode_id=observation.episode_id,
        frame_id=observation.frame_id,
        stamp=stamp,
        source_plugin_id=source_plugin_id,
    )


def image_message(payload: ImagePayload, stamp: Time, frame_id: str) -> Image:
    bytes_per_pixel = 3 if payload.encoding == "rgb8" else 4
    message = Image(
        height=payload.height,
        width=payload.width,
        encoding=payload.encoding,
        is_bigendian=0,
        step=payload.width * bytes_per_pixel,
        data=payload.data,
    )
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    return message


def pose_message(observation: Observation, stamp: Time) -> PoseStamped:
    yaw = observation.pose["yaw"]
    message = PoseStamped()
    message.header.stamp = stamp
    message.header.frame_id = "map"
    message.pose.position.x = observation.pose["x"]
    message.pose.position.y = observation.pose["y"]
    message.pose.orientation.z = math.sin(yaw / 2.0)
    message.pose.orientation.w = math.cos(yaw / 2.0)
    return message


def odometry_message(observation: Observation, stamp: Time) -> Odometry:
    yaw = observation.pose["yaw"]
    message = Odometry()
    message.header.stamp = stamp
    message.header.frame_id = "odom"
    message.child_frame_id = "base_link"
    message.pose.pose.position.x = observation.pose["x"]
    message.pose.pose.position.y = observation.pose["y"]
    message.pose.pose.orientation.z = math.sin(yaw / 2.0)
    message.pose.pose.orientation.w = math.cos(yaw / 2.0)
    return message


def goal_text_message(observation: Observation) -> String:
    return String(data=str(observation.goal["text"]))


def goal_vector_message(
    observation: Observation, stamp: Time
) -> PointStamped:
    message = PointStamped()
    message.header.stamp = stamp
    message.header.frame_id = "map"
    message.point.x = observation.goal["x"]
    message.point.y = observation.goal["y"]
    return message


def decision_payload(message: Decision) -> DecisionPayload:
    context = message.context
    common = {
        "run_id": context.run_id,
        "episode_id": context.episode_id,
        "frame_id": context.frame_id,
    }
    if message.kind == Decision.CONTINUOUS_CONTROL:
        return DecisionPayload(
            **common,
            kind="continuous_control",
            linear_x=message.continuous_control.linear.x,
            angular_z=message.continuous_control.angular.z,
        )
    if message.kind == Decision.DISCRETE_ACTION:
        actions = {1: "forward", 2: "turn_left", 3: "turn_right"}
        try:
            action = actions[message.discrete_action]
        except KeyError as error:
            raise ValueError(
                f"unsupported discrete action: {message.discrete_action}"
            ) from error
        return DecisionPayload(**common, kind="discrete_action", action=action)
    if message.kind == Decision.STOP:
        return DecisionPayload(**common, kind="stop")
    raise ValueError(f"unsupported decision kind: {message.kind}")
