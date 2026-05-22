from smart_factory.models import Pose2D
from smart_factory.reserved_axis_nav import (
    RobotAxisState,
    _frame_matches_robot,
    decide_reservations,
    should_safety_stop,
)
from smart_factory.axis_nav_to_place import build_axis_route


def _robot(robot_id, target, route_target=None, index=0, completed=False):
    robot = RobotAxisState(robot_id=robot_id, target_name=target, pose=Pose2D(0.0, 0.0, 0.0))
    if route_target is not None:
        robot.route = build_axis_route((0.0, 0.0), route_target)
        robot.waypoint_index = index
    robot.completed = completed
    return robot


def test_decide_reservations_allows_different_free_segments():
    decision = decide_reservations(
        _robot("r1", "WAIT_1", "WAIT_1", index=0),
        _robot("r2", "STACK", "STACK", index=0),
    )

    assert decision.robot_1_allowed
    assert decision.robot_2_allowed


def test_decide_reservations_reserves_same_target_for_first_robot():
    decision = decide_reservations(
        _robot("r1", "WAIT_1", "WAIT_1", index=0),
        _robot("r2", "WAIT_1", "WAIT_1", index=0),
    )

    assert decision.robot_1_allowed
    assert not decision.robot_2_allowed
    assert "reserved" in decision.reason


def test_decide_reservations_allows_only_one_robot_on_reserved_lane():
    decision = decide_reservations(
        _robot("r1", "WAIT_1", "WAIT_1", index=1),
        _robot("r2", "UNLOAD_1", "UNLOAD_1", index=1),
    )

    assert decision.robot_1_allowed
    assert not decision.robot_2_allowed
    assert "lane" in decision.reason


def test_decide_reservations_releases_lane_after_first_robot_completes():
    decision = decide_reservations(
        _robot("r1", "WAIT_1", "WAIT_1", index=1, completed=True),
        _robot("r2", "UNLOAD_1", "UNLOAD_1", index=1),
    )

    assert not decision.robot_1_allowed
    assert decision.robot_2_allowed


def test_should_safety_stop_when_robots_are_too_close():
    robot_1 = _robot("r1", "A1")
    robot_2 = _robot("r2", "A2")
    robot_2.pose = Pose2D(0.5, 0.0, 0.0)

    assert should_safety_stop(robot_1, robot_2, min_safe_distance=1.0)


def test_should_not_safety_stop_when_disabled():
    robot_1 = _robot("r1", "A1")
    robot_2 = _robot("r2", "A2")

    assert not should_safety_stop(robot_1, robot_2, min_safe_distance=0.0)


def test_frame_matches_robot_accepts_isaac_transform_tree_frames():
    assert _frame_matches_robot("chassis", "iw_hub_02", "iw_hub_02/base_link")
    assert _frame_matches_robot(
        "/World/Robots/iw_hub_02/iw_hub_sensors",
        "iw_hub_02",
        "iw_hub_02/base_link",
    )
    assert not _frame_matches_robot(
        "/World/Robots/iw_hub_01/iw_hub_sensors/front_2d_lidar",
        "iw_hub_01",
        "iw_hub_01/base_link",
    )


def test_frame_matches_robot_rejects_unqualified_global_tf_frames():
    assert not _frame_matches_robot(
        "chassis",
        "iw_hub_02",
        "iw_hub_02/base_link",
        allow_unqualified_frame=False,
    )
    assert _frame_matches_robot(
        "iw_hub_02/base_link",
        "iw_hub_02",
        "iw_hub_02/base_link",
        allow_unqualified_frame=False,
    )
