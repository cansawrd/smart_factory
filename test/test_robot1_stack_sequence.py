from smart_factory.models import Pose2D
from smart_factory.axis_nav_to_place import build_axis_route
from smart_factory.robot1_stack_sequence import (
    SequencePhase,
    _offset_pose,
    _parse_args,
    _target_for_phase,
)


def test_stack_sequence_defaults_match_robot1_topics():
    args = _parse_args([])

    assert args.odom_topic == "/iw_hub_01/odom"
    assert args.tf_topic == "/iw_hub_01/tf"
    assert args.pose_source == "tf"
    assert args.cmd_vel_topic == "/iw_hub_01/cmd_vel"
    assert args.lift_topic == "/iw_hub_01/lift_cmd"
    assert args.lift_joint_name == "lift_joint"
    assert args.lift_up_position == 0.04
    assert args.lift_down_position == 0.0
    assert args.stack_axis_order == "yx"
    assert args.stack_approach_speed == 0.5
    assert args.stack_lateral_tolerance == 0.005
    assert args.tracking_offset_x == -0.3
    assert args.tracking_offset_y == 0.0


def test_move_phases_target_requested_places():
    assert _target_for_phase(SequencePhase.MOVE_TO_STACK) == "STACK"
    assert _target_for_phase(SequencePhase.MOVE_TO_SHELF_STORAGE) == "SHELF_STORAGE"
    assert _target_for_phase(SequencePhase.MOVE_TO_UNLOAD_1) == "UNLOAD_1"


def test_offset_pose_applies_body_frame_offset():
    pose = _offset_pose(Pose2D(x=1.0, y=2.0, yaw=0.0), offset_x=-0.2, offset_y=0.1)

    assert pose.x == 0.8
    assert pose.y == 2.1
    assert pose.yaw == 0.0


def test_stack_yx_route_uses_current_y_before_final_x_approach():
    route = build_axis_route((0.5, 4.0), "STACK", axis_order="yx")

    assert route.waypoints == [(0.5, 0.0), (-13.0, 0.0)]
    assert route.axes == ["y", "x"]
