#!/usr/bin/env python3

import math
from dataclasses import dataclass
from typing import List, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
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

def duration_to_seconds(duration) -> float:
    if duration is None:
        return 0.0

    if hasattr(duration, 'nanoseconds'):
        return duration.nanoseconds / 1e9

    sec = getattr(duration, 'sec', 0)
    nanosec = getattr(duration, 'nanosec', 0)

    return float(sec) + float(nanosec) / 1e9


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


def create_pose(frame_id: str, x: float, y: float, yaw: float, node=None) -> PoseStamped:
    pose = PoseStamped()

    if frame_id is None or str(frame_id).strip() == "":
        frame_id = "map"

    pose.header.frame_id = str(frame_id)

    if node is not None:
        pose.header.stamp = node.get_clock().now().to_msg()

    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0

    qz, qw = yaw_to_quaternion_msg(yaw)
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
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


def points_to_poses(frame_id: str, points: List[Point2D], node=None) -> List[PoseStamped]:
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

        poses.append(create_pose(frame_id, cur.x, cur.y, yaw, node=node))

    return poses
def point_msg(x: float, y: float, z: float = 0.05) -> Point:
    p = Point()
    p.x = float(x)
    p.y = float(y)
    p.z = float(z)
    return p


def publish_slalom_visualization(
    node,
    path_pub,
    marker_pub,
    frame_id: str,
    poles: List[Point2D],
    nav_points: List[Point2D],
    poses: List[PoseStamped],
):
    stamp = node.get_clock().now().to_msg()

    # 1. 发布 nav_msgs/Path，RViz 里用 Path 显示
    path_msg = Path()
    path_msg.header.frame_id = frame_id
    path_msg.header.stamp = stamp

    for pose in poses:
        pose.header.frame_id = frame_id
        pose.header.stamp = stamp
        path_msg.poses.append(pose)

    path_pub.publish(path_msg)

    # 2. 发布 MarkerArray，RViz 里用 MarkerArray 显示点、杆、文字
    markers = MarkerArray()

    # 清除旧 marker
    clear_marker = Marker()
    clear_marker.header.frame_id = frame_id
    clear_marker.header.stamp = stamp
    clear_marker.action = Marker.DELETEALL
    markers.markers.append(clear_marker)

    # 2.1 杆的位置，红色球
    pole_marker = Marker()
    pole_marker.header.frame_id = frame_id
    pole_marker.header.stamp = stamp
    pole_marker.ns = "slalom_poles"
    pole_marker.id = 0
    pole_marker.type = Marker.SPHERE_LIST
    pole_marker.action = Marker.ADD
    pole_marker.scale.x = 0.12
    pole_marker.scale.y = 0.12
    pole_marker.scale.z = 0.12
    pole_marker.color.r = 1.0
    pole_marker.color.g = 0.1
    pole_marker.color.b = 0.1
    pole_marker.color.a = 1.0

    for pole in poles:
        pole_marker.points.append(point_msg(pole.x, pole.y, 0.08))

    markers.markers.append(pole_marker)

    # 2.2 计算出的绕杆点，绿色球
    point_marker = Marker()
    point_marker.header.frame_id = frame_id
    point_marker.header.stamp = stamp
    point_marker.ns = "slalom_generated_points"
    point_marker.id = 1
    point_marker.type = Marker.SPHERE_LIST
    point_marker.action = Marker.ADD
    point_marker.scale.x = 0.08
    point_marker.scale.y = 0.08
    point_marker.scale.z = 0.08
    point_marker.color.r = 0.1
    point_marker.color.g = 1.0
    point_marker.color.b = 0.1
    point_marker.color.a = 1.0

    for p in nav_points:
        point_marker.points.append(point_msg(p.x, p.y, 0.10))

    markers.markers.append(point_marker)

    # 2.3 用蓝色线把所有计算点连起来
    line_marker = Marker()
    line_marker.header.frame_id = frame_id
    line_marker.header.stamp = stamp
    line_marker.ns = "slalom_line"
    line_marker.id = 2
    line_marker.type = Marker.LINE_STRIP
    line_marker.action = Marker.ADD
    line_marker.scale.x = 0.035
    line_marker.color.r = 0.1
    line_marker.color.g = 0.4
    line_marker.color.b = 1.0
    line_marker.color.a = 1.0

    for p in nav_points:
        line_marker.points.append(point_msg(p.x, p.y, 0.04))

    markers.markers.append(line_marker)

    # 2.4 给每个路径点加文字编号 P00, P01, P02...
    for idx, p in enumerate(nav_points):
        text_marker = Marker()
        text_marker.header.frame_id = frame_id
        text_marker.header.stamp = stamp
        text_marker.ns = "slalom_point_labels"
        text_marker.id = 100 + idx
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = float(p.x)
        text_marker.pose.position.y = float(p.y)
        text_marker.pose.position.z = 0.25
        text_marker.pose.orientation.w = 1.0
        text_marker.scale.z = 0.16
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        text_marker.text = f"P{idx:02d}"
        markers.markers.append(text_marker)

    # 2.5 给杆也加编号 Pole0, Pole1...
    for idx, pole in enumerate(poles):
        text_marker = Marker()
        text_marker.header.frame_id = frame_id
        text_marker.header.stamp = stamp
        text_marker.ns = "slalom_pole_labels"
        text_marker.id = 200 + idx
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = float(pole.x)
        text_marker.pose.position.y = float(pole.y)
        text_marker.pose.position.z = 0.35
        text_marker.pose.orientation.w = 1.0
        text_marker.scale.z = 0.16
        text_marker.color.r = 1.0
        text_marker.color.g = 0.2
        text_marker.color.b = 0.2
        text_marker.color.a = 1.0
        text_marker.text = f"Pole{idx}"
        markers.markers.append(text_marker)

    marker_pub.publish(markers)

    node.get_logger().info(
        f"已发布 RViz 可视化: /slalom/debug_path 和 /slalom/debug_markers, frame_id={frame_id}"
    )

def main(args=None):
    rclpy.init(args=args)
    navigator = BasicNavigator(node_name='slalom_through_poses_runner')
    viz_qos = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )

    path_pub = navigator.create_publisher(Path, '/slalom/debug_path', viz_qos)
    marker_pub = navigator.create_publisher(MarkerArray, '/slalom/debug_markers', viz_qos)

    navigator.declare_parameter('frame_id', 'map')
    navigator.declare_parameter('offset', 0.40)
    navigator.declare_parameter('blend_distance', 0.18)
    navigator.declare_parameter('start_from_right', True)
    navigator.declare_parameter('feedback_log_period_sec', 1.0)
    navigator.declare_parameter('viz_publish_period_sec', 0.5)
    navigator.declare_parameter('pole_points', '0.0,0.4;-0.3,1.7;1.0,1.4;2.0,1.4')
    navigator.declare_parameter('start_anchor', '-0.5,0.2')
    navigator.declare_parameter('end_anchor', '2.4,1.4')

    frame_id = navigator.get_parameter('frame_id').value
    offset = float(navigator.get_parameter('offset').value)
    blend_distance = float(navigator.get_parameter('blend_distance').value)
    start_from_right = bool(navigator.get_parameter('start_from_right').value)
    feedback_log_period_sec = float(navigator.get_parameter('feedback_log_period_sec').value)
    viz_publish_period_sec = float(navigator.get_parameter('viz_publish_period_sec').value)
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
    if frame_id is None or str(frame_id).strip() == "":
        frame_id = "map"

    poses = points_to_poses(frame_id, nav_points, node=navigator)
    publish_slalom_visualization(
    node=navigator,
    path_pub=path_pub,
    marker_pub=marker_pub,
    frame_id=frame_id,
    poles=poles,
    nav_points=nav_points,
    poses=poses,
)
    navigator.get_logger().info('等待 Nav2 active...')
    navigator.waitUntilNav2Active(
    navigator='bt_navigator',
    localizer='map_server'
)

    first_side = 'right' if start_from_right else 'left'
    navigator.get_logger().info(
        f'开始执行 S 型绕杆 through-poses，points={len(poses)}，offset={offset:.2f}，blend={blend_distance:.2f}，first_side={first_side}'
    )

    for idx, pose in enumerate(poses):
        navigator.get_logger().info(
            f'P{idx:02d}: ({pose.pose.position.x:.3f}, {pose.pose.position.y:.3f})'
        )
    for idx, pose in enumerate(poses):
        navigator.get_logger().info(
            f'P{idx:02d}: frame_id="{pose.header.frame_id}", '
            f'({pose.pose.position.x:.3f}, {pose.pose.position.y:.3f})'
        )
    navigator.goThroughPoses(poses)

    last_feedback_log_time = navigator.get_clock().now()
    last_viz_pub_time = navigator.get_clock().now()

    while not navigator.isTaskComplete():
        now = navigator.get_clock().now()

        # 持续发布 RViz 可视化，防止 RViz 后添加显示项时错过消息
        if (now - last_viz_pub_time).nanoseconds / 1e9 >= viz_publish_period_sec:
            publish_slalom_visualization(
                node=navigator,
                path_pub=path_pub,
                marker_pub=marker_pub,
                frame_id=frame_id,
                poles=poles,
                nav_points=nav_points,
                poses=poses,
            )
            last_viz_pub_time = now

        feedback = navigator.getFeedback()
        if feedback is None:
            rclpy.spin_once(navigator, timeout_sec=0.05)
            continue

        if (now - last_feedback_log_time).nanoseconds / 1e9 >= feedback_log_period_sec:
            eta = 0.0
            if hasattr(feedback, 'estimated_time_remaining'):
                eta = duration_to_seconds(feedback.estimated_time_remaining)
            navigator.get_logger().info(f'任务执行中，预计剩余: {eta:.1f}s')
            last_feedback_log_time = now

        rclpy.spin_once(navigator, timeout_sec=0.05)

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
