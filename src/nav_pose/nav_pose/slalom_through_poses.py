#!/usr/bin/env python3

import math
from dataclasses import dataclass
from typing import List, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


@dataclass
class Point2D:
    x: float
    y: float


def normalize(vx: float, vy: float) -> Tuple[float, float]:
    norm = math.hypot(vx, vy)
    if norm < 1e-6:
        return 1.0, 0.0
    return vx / norm, vy / norm


def yaw_to_quaternion_msg(yaw: float):
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    return qz, qw


def parse_point_list(text: str) -> List[Point2D]:
    points: List[Point2D] = []
    chunks = [item.strip() for item in text.split(';') if item.strip()]
    for chunk in chunks:
        xy = [v.strip() for v in chunk.split(',')]
        if len(xy) != 2:
            raise ValueError(f"Invalid point format '{chunk}', expected 'x,y'")
        points.append(Point2D(float(xy[0]), float(xy[1])))
    if len(points) < 2:
        raise ValueError('Need at least 2 points in pole_points')
    return points


def parse_single_point(text: str) -> Point2D:
    xy = [v.strip() for v in text.split(',')]
    if len(xy) != 2:
        raise ValueError(f"Invalid point format '{text}', expected 'x,y'")
    return Point2D(float(xy[0]), float(xy[1]))


def create_pose(frame_id: str, x: float, y: float, yaw: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    qz, qw = yaw_to_quaternion_msg(yaw)
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose


def compute_slalom_points(
    poles: List[Point2D],
    start_anchor: Point2D,
    end_anchor: Point2D,
    offset: float,
    blend_distance: float,
    start_from_right: bool,
) -> List[Point2D]:
    generated: List[Point2D] = []

    for i, pole in enumerate(poles):
        prev_ref = start_anchor if i == 0 else poles[i - 1]
        next_ref = end_anchor if i == len(poles) - 1 else poles[i + 1]

        tx, ty = normalize(next_ref.x - prev_ref.x, next_ref.y - prev_ref.y)
        left_nx, left_ny = -ty, tx

        if start_from_right:
            side_sign = -1.0 if (i % 2 == 0) else 1.0
        else:
            side_sign = 1.0 if (i % 2 == 0) else -1.0

        apex_x = pole.x + side_sign * offset * left_nx
        apex_y = pole.y + side_sign * offset * left_ny

        entry_x = apex_x - tx * blend_distance
        entry_y = apex_y - ty * blend_distance
        exit_x = apex_x + tx * blend_distance
        exit_y = apex_y + ty * blend_distance

        if not generated:
            generated.append(Point2D(entry_x, entry_y))
        else:
            prev = generated[-1]
            if math.hypot(prev.x - entry_x, prev.y - entry_y) > 0.05:
                generated.append(Point2D(entry_x, entry_y))

        generated.append(Point2D(apex_x, apex_y))
        generated.append(Point2D(exit_x, exit_y))

    generated.append(end_anchor)
    return generated


def points_to_poses(frame_id: str, points: List[Point2D]) -> List[PoseStamped]:
    poses: List[PoseStamped] = []
    if len(points) < 2:
        return poses

    for i in range(len(points)):
        cur = points[i]
        if i < len(points) - 1:
            nxt = points[i + 1]
            yaw = math.atan2(nxt.y - cur.y, nxt.x - cur.x)
        else:
            pre = points[i - 1]
            yaw = math.atan2(cur.y - pre.y, cur.x - pre.x)
        poses.append(create_pose(frame_id, cur.x, cur.y, yaw))

    return poses


def main(args=None):
    rclpy.init(args=args)
    navigator = BasicNavigator(node_name='slalom_through_poses_runner')

    navigator.declare_parameter('frame_id', 'map')
    navigator.declare_parameter('offset', 0.40)
    navigator.declare_parameter('blend_distance', 0.18)
    navigator.declare_parameter('start_from_right', True)
    navigator.declare_parameter('feedback_log_period_sec', 1.0)
    navigator.declare_parameter('pole_points', '0.0,0.4;-0.3,1.7;1.0,1.4;2.0,1.4')
    navigator.declare_parameter('start_anchor', '-0.5,0.2')
    navigator.declare_parameter('end_anchor', '2.4,1.4')

    frame_id = navigator.get_parameter('frame_id').value
    offset = float(navigator.get_parameter('offset').value)
    blend_distance = float(navigator.get_parameter('blend_distance').value)
    start_from_right = bool(navigator.get_parameter('start_from_right').value)
    feedback_log_period_sec = float(navigator.get_parameter('feedback_log_period_sec').value)

    pole_points_str = navigator.get_parameter('pole_points').value
    start_anchor_str = navigator.get_parameter('start_anchor').value
    end_anchor_str = navigator.get_parameter('end_anchor').value

    try:
        poles = parse_point_list(pole_points_str)
        start_anchor = parse_single_point(start_anchor_str)
        end_anchor = parse_single_point(end_anchor_str)
    except ValueError as ex:
        navigator.get_logger().error(f'参数解析失败: {ex}')
        navigator.destroy_node()
        rclpy.shutdown()
        return

    nav_points = compute_slalom_points(
        poles=poles,
        start_anchor=start_anchor,
        end_anchor=end_anchor,
        offset=offset,
        blend_distance=blend_distance,
        start_from_right=start_from_right,
    )
    poses = points_to_poses(frame_id, nav_points)

    navigator.get_logger().info('等待 Nav2 active...')
    navigator.waitUntilNav2Active()

    first_side = 'right' if start_from_right else 'left'
    navigator.get_logger().info(
        f'开始执行 S 型绕杆 through-poses，points={len(poses)}，offset={offset:.2f}，blend={blend_distance:.2f}，first_side={first_side}'
    )

    for idx, pose in enumerate(poses):
        navigator.get_logger().info(
            f'P{idx:02d}: ({pose.pose.position.x:.3f}, {pose.pose.position.y:.3f})'
        )

    navigator.goThroughPoses(poses)

    last_feedback_log_time = navigator.get_clock().now()
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback is None:
            continue
        now = navigator.get_clock().now()
        if (now - last_feedback_log_time).nanoseconds / 1e9 >= feedback_log_period_sec:
            eta = 0.0
            if hasattr(feedback, 'estimated_time_remaining'):
                eta = feedback.estimated_time_remaining.nanoseconds / 1e9
            navigator.get_logger().info(f'任务执行中，预计剩余: {eta:.1f}s')
            last_feedback_log_time = now

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        navigator.get_logger().info('S 型绕杆任务完成 ✅')
    elif result == TaskResult.CANCELED:
        navigator.get_logger().warn('S 型绕杆任务被取消')
    else:
        navigator.get_logger().error('S 型绕杆任务失败 ❌')

    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
