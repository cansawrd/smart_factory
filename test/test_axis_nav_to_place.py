import pytest

from smart_factory.axis_nav_to_place import (
    PLACE_CANDIDATES,
    PLACES,
    build_axis_route,
    compute_axis_nav_command,
)
from smart_factory.models import Pose2D


def test_places_define_requested_targets():
    assert PLACE_CANDIDATES["WAIT"] == [(12.0, 15.0), (12.0, 10.0), (12.0, 7.0)]
    assert PLACE_CANDIDATES["STACK"] == [(-13.0, 0.0)]
    assert PLACE_CANDIDATES["SHELF_STORAGE"] == [(0.0, -10.0)]
    assert PLACE_CANDIDATES["UNLOAD"] == [(13.0, -10.0), (13.0, 0.0), (13.0, 10.0)]
    assert PLACES["WAIT_1"] == (12.0, 15.0)
    assert PLACES["WAIT_2"] == (12.0, 10.0)
    assert PLACES["WAIT_3"] == (12.0, 7.0)
    assert PLACES["STACK"] == (-13.0, 0.0)
    assert PLACES["SHELF_STORAGE"] == (0.0, -10.0)
    assert PLACES["UNLOAD_1"] == (13.0, -10.0)
    assert PLACES["UNLOAD_2"] == (13.0, 0.0)
    assert PLACES["UNLOAD_3"] == (13.0, 10.0)


def test_build_axis_route_uses_x_then_y_segments():
    route = build_axis_route((0.5, -1.0), "STACK", axis_order="xy")

    assert route.target_name == "STACK"
    assert route.waypoints == [(-13.0, -1.0), (-13.0, 0.0)]
    assert route.axes == ["x", "y"]


def test_build_axis_route_uses_y_then_x_segments():
    route = build_axis_route((0.5, -1.0), "STACK", axis_order="yx")

    assert route.waypoints == [(0.5, 0.0), (-13.0, 0.0)]
    assert route.axes == ["y", "x"]


def test_build_axis_route_removes_zero_length_segments():
    route = build_axis_route((12.0, 8.0), "WAIT_3", axis_order="xy")

    assert route.waypoints == [(12.0, 7.0)]
    assert route.axes == ["y"]


def test_build_axis_route_removes_tiny_zero_length_segments():
    route = build_axis_route((13.0, -0.0004), "UNLOAD_2", axis_order="xy")

    assert route.waypoints == []
    assert route.axes == []


def test_build_axis_route_uses_nearest_candidate_for_place_group():
    route = build_axis_route((12.0, 9.0), "WAIT", axis_order="xy")

    assert route.target_name == "WAIT_2"
    assert route.waypoints == [(12.0, 10.0)]
    assert route.axes == ["y"]


def test_build_axis_route_rejects_unknown_target():
    with pytest.raises(ValueError):
        build_axis_route((0.0, 0.0), "A4")


def test_compute_axis_nav_command_drives_when_facing_target():
    command = compute_axis_nav_command(
        Pose2D(x=0.0, y=0.0, yaw=0.0),
        (1.0, 0.0),
        segment_start=(0.0, 0.0),
        active_axis="x",
        max_linear_speed=0.35,
        max_angular_speed=1.0,
        distance_tolerance=0.1,
        yaw_tolerance=0.2,
    )

    assert command.linear_x == 0.35
    assert command.angular_z == 0.0
    assert not command.done


def test_compute_axis_nav_command_uses_yaw_offset():
    command = compute_axis_nav_command(
        Pose2D(x=0.0, y=0.0, yaw=3.141592653589793),
        (1.0, 0.0),
        segment_start=(0.0, 0.0),
        active_axis="x",
        max_linear_speed=0.35,
        max_angular_speed=1.0,
        distance_tolerance=0.1,
        yaw_tolerance=0.2,
        yaw_offset=3.141592653589793,
    )

    assert command.linear_x == 0.35
    assert abs(command.yaw_error) < 1e-6


def test_compute_axis_nav_command_applies_angular_sign():
    command = compute_axis_nav_command(
        Pose2D(x=0.0, y=0.0, yaw=0.0),
        (0.0, 1.0),
        segment_start=(0.0, 0.0),
        active_axis="y",
        max_linear_speed=0.35,
        max_angular_speed=1.0,
        distance_tolerance=0.1,
        yaw_tolerance=0.2,
        angular_sign=-1.0,
    )

    assert command.linear_x == 0.0
    assert command.angular_z == -1.0


def test_compute_axis_nav_command_finishes_when_axis_target_crossed():
    command = compute_axis_nav_command(
        Pose2D(x=3.2, y=5.0, yaw=0.0),
        (3.0, 1.0),
        segment_start=(0.0, 5.0),
        active_axis="x",
        max_linear_speed=0.35,
        max_angular_speed=1.0,
        distance_tolerance=0.1,
        yaw_tolerance=0.2,
    )

    assert command.done
    assert command.crossed_axis_target
