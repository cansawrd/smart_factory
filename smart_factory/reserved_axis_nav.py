from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from typing import Optional

from smart_factory.axis_nav_to_place import (
    AxisRoute,
    PLACES,
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
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
except ImportError:  # Allows algorithm tests without a sourced ROS2 environment.
    rclpy = None
    Twist = object
    Odometry = object
    Node = object
    QoSProfile = object
    String = object
    TFMessage = object


@dataclass
class RobotAxisState:
    robot_id: str
    target_name: str
    pose: Pose2D | None = None
    route: AxisRoute | None = None
    waypoint_index: int = 0
    segment_start_pose: Pose2D | None = None
    completed: bool = False
    pose_source: str | None = None

    @property
    def active_axis(self) -> str | None:
        if self.route is None or self.waypoint_index >= len(self.route.axes):
            return None
        return self.route.axes[self.waypoint_index]

    @property
    def is_on_reserved_lane(self) -> bool:
        return self.active_axis == "y"


@dataclass(frozen=True)
class ReservationDecision:
    robot_1_allowed: bool
    robot_2_allowed: bool
    reason: str


def decide_reservations(robot_1: RobotAxisState, robot_2: RobotAxisState) -> ReservationDecision:
    if robot_1.completed and robot_2.completed:
        return ReservationDecision(False, False, "complete")

    if robot_1.target_name == robot_2.target_name:
        if not robot_1.completed:
            return ReservationDecision(True, False, f"{robot_2.robot_id} waits: {robot_1.target_name} reserved")
        return ReservationDecision(False, False, f"{robot_2.target_name} occupied by {robot_1.robot_id}")

    if robot_1.is_on_reserved_lane and robot_2.is_on_reserved_lane:
        if not robot_1.completed:
            return ReservationDecision(True, False, f"{robot_2.robot_id} waits: x=3 lane reserved")
        return ReservationDecision(False, True, f"{robot_1.robot_id} completed lane")

    return ReservationDecision(not robot_1.completed, not robot_2.completed, "free")


def distance_between(left: Pose2D, right: Pose2D) -> float:
    return math.hypot(left.x - right.x, left.y - right.y)


def should_safety_stop(robot_1: RobotAxisState, robot_2: RobotAxisState, min_safe_distance: float) -> bool:
    if min_safe_distance <= 0.0 or robot_1.pose is None or robot_2.pose is None:
        return False
    return distance_between(robot_1.pose, robot_2.pose) < min_safe_distance


class ReservedAxisNav(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("smart_factory_reserved_axis_nav")
        self.args = args
        self.robot_1 = RobotAxisState(args.robot_1_id, args.robot_1_target.upper())
        self.robot_2 = RobotAxisState(args.robot_2_id, args.robot_2_target.upper())
        self.last_wait_log_time: float | None = None

        isaac_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.robot_1_pub = self.create_publisher(Twist, args.robot_1_cmd_vel, 10)
        self.robot_2_pub = self.create_publisher(Twist, args.robot_2_cmd_vel, 10)
        self.status_pub = self.create_publisher(String, "/smart_factory/reserved_axis_nav_status", 10)
        self._subscriptions = [
            self.create_subscription(Odometry, args.robot_1_odom, self._on_robot_1_odom, isaac_qos),
            self.create_subscription(Odometry, args.robot_2_odom, self._on_robot_2_odom, isaac_qos),
            self.create_subscription(TFMessage, args.robot_1_tf, self._on_robot_1_tf, isaac_qos),
            self.create_subscription(TFMessage, args.robot_2_tf, self._on_robot_2_tf, isaac_qos),
        ]
        self.timer = self.create_timer(1.0 / args.rate, self._on_timer)
        self.get_logger().info(
            f"Reserved axis nav: {self.robot_1.robot_id}->{self.robot_1.target_name}, "
            f"{self.robot_2.robot_id}->{self.robot_2.target_name}; "
            f"pose_source={args.pose_source}; "
            f"tf=({args.robot_1_tf}, {args.robot_2_tf}); "
            f"odom=({args.robot_1_odom}, {args.robot_2_odom})"
        )

    def _on_robot_1_odom(self, msg: Odometry) -> None:
        if self.args.pose_source in {"odom", "auto"} and self.robot_1.pose_source != "tf":
            self.robot_1.pose = _pose_from_odom(msg)
            self.robot_1.pose_source = "odom"

    def _on_robot_2_odom(self, msg: Odometry) -> None:
        if self.args.pose_source in {"odom", "auto"} and self.robot_2.pose_source != "tf":
            self.robot_2.pose = _pose_from_odom(msg)
            self.robot_2.pose_source = "odom"

    def _on_robot_1_tf(self, msg: TFMessage) -> None:
        self._on_robot_tf(self.robot_1, msg, self.args.robot_1_base_frame, self.args.robot_1_tf)

    def _on_robot_2_tf(self, msg: TFMessage) -> None:
        self._on_robot_tf(self.robot_2, msg, self.args.robot_2_base_frame, self.args.robot_2_tf)

    def _on_robot_tf(
        self,
        robot: RobotAxisState,
        msg: TFMessage,
        base_frame: str,
        tf_topic: str,
    ) -> None:
        if self.args.pose_source not in {"tf", "auto"}:
            return
        allow_unqualified_frame = tf_topic != "/tf"
        for transform in msg.transforms:
            if _frame_matches_robot(
                transform.child_frame_id,
                robot.robot_id,
                base_frame,
                allow_unqualified_frame=allow_unqualified_frame,
            ):
                robot.pose = _pose_from_transform(transform)
                robot.pose_source = "tf"
                return

    def _on_timer(self) -> None:
        if self.robot_1.pose is None or self.robot_2.pose is None:
            self._publish_waiting_for_poses()
            return

        self._ensure_routes()
        if should_safety_stop(self.robot_1, self.robot_2, self.args.min_safe_distance):
            robot_distance = distance_between(self.robot_1.pose, self.robot_2.pose)
            self._publish_twists(0.0, 0.0, 0.0, 0.0)
            self._publish_status(
                f"safety_stop robot_distance={robot_distance:.3f} "
                f"min_safe_distance={self.args.min_safe_distance:.3f}"
            )
            return

        decision = decide_reservations(self.robot_1, self.robot_2)

        robot_1_linear, robot_1_angular, robot_1_text = self._step_robot(self.robot_1, decision.robot_1_allowed)
        robot_2_linear, robot_2_angular, robot_2_text = self._step_robot(self.robot_2, decision.robot_2_allowed)
        self._publish_twists(robot_1_linear, robot_1_angular, robot_2_linear, robot_2_angular)
        self._publish_status(f"reservation={decision.reason}; {robot_1_text}; {robot_2_text}")

    def _ensure_routes(self) -> None:
        for robot in (self.robot_1, self.robot_2):
            if robot.route is None and robot.pose is not None:
                robot.route = build_axis_route(
                    (robot.pose.x, robot.pose.y),
                    robot.target_name,
                    axis_order=self.args.axis_order,
                )
                self.get_logger().info(
                    f"{robot.robot_id} route to {robot.target_name}: "
                    + " -> ".join(f"({x:.3f},{y:.3f})" for x, y in robot.route.waypoints)
                )

    def _step_robot(self, robot: RobotAxisState, allowed: bool) -> tuple[float, float, str]:
        if robot.route is None or robot.pose is None:
            return 0.0, 0.0, f"{robot.robot_id}=waiting_route"
        if robot.completed or robot.waypoint_index >= len(robot.route.waypoints):
            robot.completed = True
            return 0.0, 0.0, f"{robot.robot_id}=complete target={robot.target_name}"
        if not allowed:
            return 0.0, 0.0, f"{robot.robot_id}=reserved_wait target={robot.target_name}"

        if robot.segment_start_pose is None:
            robot.segment_start_pose = robot.pose

        target = robot.route.waypoints[robot.waypoint_index]
        active_axis = robot.route.axes[robot.waypoint_index]
        command = compute_axis_nav_command(
            robot.pose,
            target,
            segment_start=(robot.segment_start_pose.x, robot.segment_start_pose.y),
            active_axis=active_axis,
            max_linear_speed=self.args.speed,
            max_angular_speed=self.args.turn_speed,
            distance_tolerance=self.args.distance_tolerance,
            yaw_tolerance=self.args.yaw_tolerance,
            yaw_offset=self.args.yaw_offset,
            angular_sign=self.args.angular_sign,
        )

        if command.done:
            robot.waypoint_index += 1
            robot.segment_start_pose = None
            if robot.waypoint_index >= len(robot.route.waypoints):
                robot.completed = True

        text = (
            f"{robot.robot_id}=move target={robot.target_name} wp={robot.waypoint_index}/"
            f"{len(robot.route.waypoints)} axis={active_axis} "
            f"x={robot.pose.x:.3f} y={robot.pose.y:.3f} source={robot.pose_source} "
            f"linear={command.linear_x:.3f} angular={command.angular_z:.3f} done={command.done}"
        )
        return command.linear_x, command.angular_z, text

    def _publish_twists(
        self,
        robot_1_linear: float,
        robot_1_angular: float,
        robot_2_linear: float,
        robot_2_angular: float,
    ) -> None:
        robot_1_command = Twist()
        robot_1_command.linear.x = robot_1_linear
        robot_1_command.angular.z = robot_1_angular
        self.robot_1_pub.publish(robot_1_command)

        robot_2_command = Twist()
        robot_2_command.linear.x = robot_2_linear
        robot_2_command.angular.z = robot_2_angular
        self.robot_2_pub.publish(robot_2_command)

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def _publish_waiting_for_poses(self) -> None:
        missing = [
            robot.robot_id
            for robot in (self.robot_1, self.robot_2)
            if robot.pose is None
        ]
        text = (
            f"waiting for poses missing={','.join(missing)} "
            f"source={self.args.pose_source} "
            f"tf=({self.args.robot_1_tf},{self.args.robot_2_tf}) "
            f"odom=({self.args.robot_1_odom},{self.args.robot_2_odom})"
        )
        self._publish_status(text)

        now = self.get_clock().now().nanoseconds / 1_000_000_000.0
        if self.last_wait_log_time is None or now - self.last_wait_log_time >= 2.0:
            self.last_wait_log_time = now
            self.get_logger().info(text)

    def _try_publish_stop(self) -> None:
        try:
            self._publish_twists(0.0, 0.0, 0.0, 0.0)
        except Exception as exc:  # ROS may invalidate the context before KeyboardInterrupt is handled.
            self.get_logger().debug(f"Stop publish skipped during shutdown: {exc}")


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


def _frame_matches_robot(
    child_frame_id: str,
    robot_id: str,
    base_frame: str,
    *,
    allow_unqualified_frame: bool = True,
) -> bool:
    if child_frame_id == base_frame or child_frame_id.endswith(f"/{base_frame}"):
        return True
    if allow_unqualified_frame and child_frame_id in {"chassis", "base_link", "iw_hub_sensors"}:
        return True
    if robot_id not in child_frame_id:
        return False
    frame_tail = child_frame_id.rsplit("/", maxsplit=1)[-1]
    return frame_tail in {"base_link", "iw_hub_sensors"}


def _parse_args(args: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Two-robot axis navigation with named place targets and lane reservations."
    )
    parser.add_argument("--robot-1-target", choices=sorted(PLACES), required=True)
    parser.add_argument("--robot-2-target", choices=sorted(PLACES), required=True)
    parser.add_argument("--axis-order", choices=["xy", "yx"], default="xy")
    parser.add_argument("--robot-1-id", default=default_robot_id(1))
    parser.add_argument("--robot-2-id", default=default_robot_id(2))
    parser.add_argument("--robot-1-odom", default=default_odom_topic(1))
    parser.add_argument("--robot-2-odom", default=default_odom_topic(2))
    parser.add_argument("--robot-1-cmd-vel", default=default_cmd_vel_topic(1))
    parser.add_argument("--robot-2-cmd-vel", default=default_cmd_vel_topic(2))
    parser.add_argument("--robot-1-tf", default=f"/{default_robot_id(1)}/tf")
    parser.add_argument("--robot-2-tf", default=f"/{default_robot_id(2)}/tf")
    parser.add_argument("--robot-1-base-frame", default=default_base_frame(1))
    parser.add_argument("--robot-2-base-frame", default=default_base_frame(2))
    parser.add_argument(
        "--pose-source",
        choices=["tf", "odom", "auto"],
        default="tf",
        help="Use tf for world poses, odom for robot-local poses, or auto with tf overriding odom.",
    )
    parser.add_argument("--speed", type=float, default=2.0)
    parser.add_argument("--turn-speed", type=float, default=2.0)
    parser.add_argument("--distance-tolerance", type=float, default=0.2)
    parser.add_argument("--yaw-tolerance", type=float, default=0.2)
    parser.add_argument(
        "--min-safe-distance",
        type=float,
        default=1.2,
        help="Stop both robots when their odom positions are closer than this distance. Use 0 to disable.",
    )
    parser.add_argument("--yaw-offset", type=float, default=0.0)
    parser.add_argument("--angular-sign", type=float, choices=[-1.0, 1.0], default=1.0)
    parser.add_argument("--rate", type=float, default=10.0)
    return parser.parse_args(args)


def main(args: Optional[list[str]] = None) -> None:
    if rclpy is None:
        raise RuntimeError("rclpy is not available. Source ROS2 before running reserved_axis_nav.")

    parsed_args = _parse_args(args)
    rclpy.init(args=args)
    node = ReservedAxisNav(parsed_args)
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
