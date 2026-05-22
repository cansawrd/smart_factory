from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from enum import Enum
from typing import Optional

from smart_factory.axis_nav_to_place import (
    AxisRoute,
    build_axis_route,
    compute_axis_nav_command,
)
from smart_factory.models import Pose2D
from smart_factory.pose_estimator import yaw_from_quaternion
from smart_factory.robot_defaults import (
    default_base_frame,
    default_cmd_vel_topic,
    default_odom_topic,
    default_robot_id,
)

try:
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
except ImportError:  # Allows algorithm tests without a sourced ROS2 environment.
    rclpy = None
    Twist = object
    Odometry = object
    Node = object
    QoSProfile = object
    JointState = object
    String = object
    TFMessage = object


class SequencePhase(Enum):
    MOVE_TO_STACK = "move_to_stack"
    LIFT_UP = "lift_up"
    WAIT_AFTER_LIFT = "wait_after_lift"
    MOVE_TO_SHELF_STORAGE = "move_to_shelf_storage"
    STOP_AT_SHELF_STORAGE = "stop_at_shelf_storage"
    MOVE_TO_UNLOAD_1 = "move_to_unload_1"
    LIFT_DOWN = "lift_down"
    COMPLETE = "complete"


@dataclass
class MoveState:
    target_name: str
    route: AxisRoute | None = None
    waypoint_index: int = 0
    segment_start_pose: Pose2D | None = None


class Robot1StackSequence(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("smart_factory_robot1_stack_sequence")
        self.args = args
        self.odom_pose: Pose2D | None = None
        self.tf_pose: Pose2D | None = None
        self.pose_source: str | None = None
        self.pose: Pose2D | None = None
        self.phase = SequencePhase.MOVE_TO_STACK
        self.phase_started_at = self._now()
        self.move_state = MoveState("STACK")

        isaac_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.cmd_pub = self.create_publisher(Twist, args.cmd_vel_topic, 10)
        self.lift_pub = self.create_publisher(JointState, args.lift_topic, 10)
        self.status_pub = self.create_publisher(String, args.status_topic, 10)
        self._odom_subscription = self.create_subscription(
            Odometry,
            args.odom_topic,
            self._on_odom,
            isaac_qos,
        )
        self._tf_subscription = self.create_subscription(
            TFMessage,
            args.tf_topic,
            self._on_tf,
            isaac_qos,
        )
        self.timer = self.create_timer(1.0 / args.rate, self._on_timer)
        self.get_logger().info(
            "Robot1 stack sequence: STACK -> lift up -> wait -> "
            "SHELF_STORAGE -> stop -> UNLOAD_1 -> lift down; "
            f"pose_source={args.pose_source}"
        )

    def _on_odom(self, msg: Odometry) -> None:
        self.odom_pose = _pose_from_odom(msg)
        if self.args.pose_source == "odom" or (
            self.args.pose_source == "auto" and self.pose_source != "tf"
        ):
            self._set_pose(self.odom_pose, "odom")

    def _on_tf(self, msg: TFMessage) -> None:
        if self.args.pose_source not in {"tf", "auto"}:
            return
        for transform in msg.transforms:
            if _is_world_to_robot_base(transform, self.args.base_frame):
                self.tf_pose = _pose_from_transform(transform)
                self._set_pose(self.tf_pose, "tf")
                return

    def _set_pose(self, pose: Pose2D, source: str) -> None:
        self.pose = _offset_pose(
            pose,
            offset_x=self.args.tracking_offset_x,
            offset_y=self.args.tracking_offset_y,
        )
        self.pose_source = source

    def _on_timer(self) -> None:
        if self.pose is None:
            self._publish_stop()
            self._publish_status("waiting for odom")
            return

        if self.phase == SequencePhase.MOVE_TO_STACK:
            if self._step_move("STACK"):
                self._change_phase(SequencePhase.LIFT_UP)
            return

        if self.phase == SequencePhase.LIFT_UP:
            self._publish_stop()
            self._publish_lift(self.args.lift_up_position)
            self._change_phase(SequencePhase.WAIT_AFTER_LIFT)
            return

        if self.phase == SequencePhase.WAIT_AFTER_LIFT:
            self._publish_stop()
            self._publish_lift(self.args.lift_up_position)
            if self._elapsed() >= self.args.wait_after_lift:
                self._change_phase(SequencePhase.MOVE_TO_SHELF_STORAGE)
            else:
                self._publish_status(f"phase={self.phase.value}; waiting={self._elapsed():.2f}")
            return

        if self.phase == SequencePhase.MOVE_TO_SHELF_STORAGE:
            self._publish_lift(self.args.lift_up_position)
            if self._step_move("SHELF_STORAGE"):
                self._change_phase(SequencePhase.STOP_AT_SHELF_STORAGE)
            return

        if self.phase == SequencePhase.STOP_AT_SHELF_STORAGE:
            self._publish_stop()
            self._publish_lift(self.args.lift_up_position)
            if self._elapsed() >= self.args.shelf_stop_duration:
                self._change_phase(SequencePhase.MOVE_TO_UNLOAD_1)
            else:
                self._publish_status(f"phase={self.phase.value}; stopped={self._elapsed():.2f}")
            return

        if self.phase == SequencePhase.MOVE_TO_UNLOAD_1:
            self._publish_lift(self.args.lift_up_position)
            if self._step_move("UNLOAD_1"):
                self._change_phase(SequencePhase.LIFT_DOWN)
            return

        if self.phase == SequencePhase.LIFT_DOWN:
            self._publish_stop()
            self._publish_lift(self.args.lift_down_position)
            if self._elapsed() >= self.args.lift_down_hold:
                self._change_phase(SequencePhase.COMPLETE)
            else:
                self._publish_status(f"phase={self.phase.value}; lowering={self._elapsed():.2f}")
            return

        self._publish_stop()
        self._publish_lift(self.args.lift_down_position)
        self._publish_status("phase=complete")

    def _step_move(self, target_name: str) -> bool:
        if self.move_state.target_name != target_name:
            self.move_state = MoveState(target_name)

        if self.move_state.route is None:
            self.move_state.route = build_axis_route(
                (self.pose.x, self.pose.y),
                target_name,
                axis_order=self._axis_order_for_target(target_name),
            )
            self.get_logger().info(
                f"Route to {self.move_state.route.target_name}: "
                + " -> ".join(f"({x:.3f},{y:.3f})" for x, y in self.move_state.route.waypoints)
            )
            self._publish_status(
                f"phase={self.phase.value}; target={target_name}; route="
                + "->".join(
                    f"({x:.3f},{y:.3f})/{axis}"
                    for (x, y), axis in zip(self.move_state.route.waypoints, self.move_state.route.axes)
                )
            )

        if self.move_state.waypoint_index >= len(self.move_state.route.waypoints):
            self._publish_stop()
            self._publish_status(f"phase={self.phase.value}; target={target_name}; done=True")
            return True

        if self.move_state.segment_start_pose is None:
            self.move_state.segment_start_pose = self.pose

        target = self.move_state.route.waypoints[self.move_state.waypoint_index]
        active_axis = self.move_state.route.axes[self.move_state.waypoint_index]
        command = compute_axis_nav_command(
            self.pose,
            target,
            segment_start=(
                self.move_state.segment_start_pose.x,
                self.move_state.segment_start_pose.y,
            ),
            active_axis=active_axis,
            max_linear_speed=self._speed_for_segment(target_name, active_axis),
            max_angular_speed=self.args.turn_speed,
            distance_tolerance=self._distance_tolerance_for_segment(target_name, active_axis),
            yaw_tolerance=self.args.yaw_tolerance,
            yaw_offset=self.args.yaw_offset,
            angular_sign=self.args.angular_sign,
        )
        self._publish_command(command.linear_x, command.angular_z)
        self._publish_status(
            f"phase={self.phase.value}; target={target_name}; waypoint="
            f"{self.move_state.waypoint_index + 1}/{len(self.move_state.route.waypoints)}; "
            f"odom_x={_format_pose_value(self.odom_pose, 'x')}; "
            f"odom_y={_format_pose_value(self.odom_pose, 'y')}; "
            f"tf_x={_format_pose_value(self.tf_pose, 'x')}; "
            f"tf_y={_format_pose_value(self.tf_pose, 'y')}; source={self.pose_source}; "
            f"track_x={self.pose.x:.3f}; track_y={self.pose.y:.3f}; axis={active_axis}; "
            f"goal=({target[0]:.3f},{target[1]:.3f}); "
            f"axis_error={command.axis_error:.3f}; tolerance="
            f"{self._distance_tolerance_for_segment(target_name, active_axis):.3f}; "
            f"target_yaw={command.target_yaw:.3f}; control_yaw={command.control_yaw:.3f}; "
            f"yaw_error={command.yaw_error:.3f}; "
            f"linear={command.linear_x:.3f}; angular={command.angular_z:.3f}; "
            f"done={command.done}"
        )

        if command.done:
            self.move_state.waypoint_index += 1
            self.move_state.segment_start_pose = None
        return False

    def _axis_order_for_target(self, target_name: str) -> str:
        if target_name == "STACK":
            return self.args.stack_axis_order
        return self.args.axis_order

    def _speed_for_segment(self, target_name: str, active_axis: str) -> float:
        if target_name == "STACK" and active_axis == "x":
            return self.args.stack_approach_speed
        return self.args.speed

    def _distance_tolerance_for_segment(self, target_name: str, active_axis: str) -> float:
        if target_name == "STACK" and active_axis == "y":
            return self.args.stack_lateral_tolerance
        return self.args.distance_tolerance

    def _change_phase(self, phase: SequencePhase) -> None:
        self.phase = phase
        self.phase_started_at = self._now()
        self.move_state = MoveState(_target_for_phase(phase))
        self._publish_status(f"phase={self.phase.value}")

    def _elapsed(self) -> float:
        return self._now() - self.phase_started_at

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _publish_command(self, linear_x: float, angular_z: float) -> None:
        command = Twist()
        command.linear.x = linear_x
        command.angular.z = angular_z
        self.cmd_pub.publish(command)

    def _publish_stop(self) -> None:
        self._publish_command(0.0, 0.0)

    def _publish_lift(self, position: float) -> None:
        command = JointState()
        command.name = [self.args.lift_joint_name]
        command.position = [position]
        self.lift_pub.publish(command)

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def _try_publish_stop(self) -> None:
        try:
            self._publish_stop()
        except Exception as exc:  # ROS may invalidate the context before shutdown handling.
            self.get_logger().debug(f"Stop publish skipped during shutdown: {exc}")


def _target_for_phase(phase: SequencePhase) -> str:
    if phase == SequencePhase.MOVE_TO_STACK:
        return "STACK"
    if phase == SequencePhase.MOVE_TO_SHELF_STORAGE:
        return "SHELF_STORAGE"
    if phase == SequencePhase.MOVE_TO_UNLOAD_1:
        return "UNLOAD_1"
    return ""


def _pose_from_odom(msg: Odometry) -> Pose2D:
    position = msg.pose.pose.position
    orientation = msg.pose.pose.orientation
    return Pose2D(
        x=position.x,
        y=position.y,
        yaw=yaw_from_quaternion(orientation.x, orientation.y, orientation.z, orientation.w),
    )


def _pose_from_transform(transform) -> Pose2D:
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    return Pose2D(
        x=translation.x,
        y=translation.y,
        yaw=yaw_from_quaternion(rotation.x, rotation.y, rotation.z, rotation.w),
    )


def _is_world_to_robot_base(transform, base_frame: str) -> bool:
    if transform.header.frame_id != "world":
        return False
    child_frame = transform.child_frame_id
    if child_frame == base_frame or child_frame.endswith(f"/{base_frame}"):
        return True
    frame_tail = child_frame.rsplit("/", maxsplit=1)[-1]
    base_tail = base_frame.rsplit("/", maxsplit=1)[-1]
    return frame_tail in {"chassis", "base_link", "iw_hub_sensors", base_tail}


def _format_pose_value(pose: Pose2D | None, attr_name: str) -> str:
    if pose is None:
        return "nan"
    return f"{getattr(pose, attr_name):.3f}"


def _offset_pose(pose: Pose2D, *, offset_x: float, offset_y: float) -> Pose2D:
    if offset_x == 0.0 and offset_y == 0.0:
        return pose
    cos_yaw = math.cos(pose.yaw)
    sin_yaw = math.sin(pose.yaw)
    return Pose2D(
        x=pose.x + offset_x * cos_yaw - offset_y * sin_yaw,
        y=pose.y + offset_x * sin_yaw + offset_y * cos_yaw,
        yaw=pose.yaw,
    )


def _parse_args(args: Optional[list[str]] = None) -> argparse.Namespace:
    robot_id = default_robot_id(1)
    parser = argparse.ArgumentParser(
        description="Run robot1 through stack pickup, shelf stop, and unload dropoff."
    )
    parser.add_argument("--odom-topic", default=default_odom_topic(1))
    parser.add_argument("--tf-topic", default=f"/{robot_id}/tf")
    parser.add_argument("--base-frame", default=default_base_frame(1))
    parser.add_argument(
        "--pose-source",
        choices=["tf", "odom", "auto"],
        default="tf",
        help="Use tf for world poses, odom for robot-local odometry, or auto with tf overriding odom.",
    )
    parser.add_argument("--cmd-vel-topic", default=default_cmd_vel_topic(1))
    parser.add_argument("--lift-topic", default=f"/{robot_id}/lift_cmd")
    parser.add_argument("--status-topic", default="/smart_factory/robot1_stack_sequence_status")
    parser.add_argument("--axis-order", choices=["xy", "yx"], default="xy")
    parser.add_argument(
        "--stack-axis-order",
        choices=["xy", "yx"],
        default="yx",
        help="Use yx to align left/right before the final straight stack approach.",
    )
    parser.add_argument("--speed", type=float, default=2.0)
    parser.add_argument("--stack-approach-speed", type=float, default=0.5)
    parser.add_argument(
        "--stack-lateral-tolerance",
        type=float,
        default=0.005,
        help="Y-axis tolerance used before the final stack approach.",
    )
    parser.add_argument("--turn-speed", type=float, default=2.0)
    parser.add_argument("--distance-tolerance", type=float, default=0.12)
    parser.add_argument("--yaw-tolerance", type=float, default=0.2)
    parser.add_argument("--yaw-offset", type=float, default=0.0)
    parser.add_argument("--angular-sign", type=float, choices=[-1.0, 1.0], default=1.0)
    parser.add_argument("--lift-joint-name", default="lift_joint")
    parser.add_argument("--lift-up-position", type=float, default=0.04)
    parser.add_argument("--lift-down-position", type=float, default=0.0)
    parser.add_argument("--wait-after-lift", type=float, default=1.0)
    parser.add_argument("--shelf-stop-duration", type=float, default=1.0)
    parser.add_argument("--lift-down-hold", type=float, default=1.0)
    parser.add_argument(
        "--tracking-offset-x",
        type=float,
        default=-0.3,
        help="Body-frame x offset from odom/chassis to the point that should reach each target.",
    )
    parser.add_argument(
        "--tracking-offset-y",
        type=float,
        default=0.0,
        help="Body-frame y offset from odom/chassis to the point that should reach each target.",
    )
    parser.add_argument("--rate", type=float, default=10.0)
    return parser.parse_args(args)


def main(args: Optional[list[str]] = None) -> None:
    if rclpy is None:
        raise RuntimeError("rclpy is not available. Source ROS2 before running robot1_stack_sequence.")

    parsed_args = _parse_args(args)
    rclpy.init(args=args)
    node = Robot1StackSequence(parsed_args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if rclpy.ok():
            node._try_publish_stop()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
