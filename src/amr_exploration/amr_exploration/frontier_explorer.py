import math
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')

        self.declare_parameter('centroid_topic', '/frontier_centroids')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('goal_timeout_sec', 60.0)
        self.declare_parameter('reached_tolerance', 0.40)
        self.declare_parameter('blacklist_tolerance', 0.50)
        self.declare_parameter('min_frontier_distance', 0.30)

        centroid_topic = self.get_parameter('centroid_topic').value

        self.frontiers: List[Tuple[float, float]] = []
        self.visited_frontiers: List[Tuple[float, float]] = []
        self.blacklisted_frontiers: List[Tuple[float, float]] = []

        self.goal_active = False
        self.current_goal: Optional[Tuple[float, float]] = None
        self.goal_start_time = None
        self.result_future = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.centroid_sub = self.create_subscription(
            Marker,
            centroid_topic,
            self.centroids_callback,
            10
        )

        self.timer = self.create_timer(1.0, self.control_loop)

        self.get_logger().info('Frontier explorer started.')
        self.get_logger().info(f'Subscribing to centroid topic: {centroid_topic}')

    def centroids_callback(self, msg: Marker):
        new_frontiers = []
        for p in msg.points:
            new_frontiers.append((p.x, p.y))

        self.frontiers = new_frontiers

    def get_robot_position(self) -> Optional[Tuple[float, float]]:
        map_frame = self.get_parameter('map_frame').value
        base_frame = self.get_parameter('base_frame').value

        try:
            transform = self.tf_buffer.lookup_transform(
                map_frame,
                base_frame,
                rclpy.time.Time()
            )
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            return (x, y)

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'Could not get robot pose: {str(e)}')
            return None

    def distance(self, a: Tuple[float, float], b: Tuple[float, float]) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def is_near_any(
        self,
        point: Tuple[float, float],
        point_list: List[Tuple[float, float]],
        tolerance: float
    ) -> bool:
        for p in point_list:
            if self.distance(point, p) <= tolerance:
                return True
        return False

    def choose_best_frontier(
        self,
        robot_pos: Tuple[float, float]
    ) -> Optional[Tuple[float, float]]:
        min_frontier_distance = self.get_parameter('min_frontier_distance').value
        reached_tolerance = self.get_parameter('reached_tolerance').value
        blacklist_tolerance = self.get_parameter('blacklist_tolerance').value

        candidates = []

        for frontier in self.frontiers:
            dist = self.distance(robot_pos, frontier)

            if dist < min_frontier_distance:
                continue

            if self.is_near_any(frontier, self.visited_frontiers, reached_tolerance):
                continue

            if self.is_near_any(frontier, self.blacklisted_frontiers, blacklist_tolerance):
                continue

            candidates.append((dist, frontier))

        if not candidates:
            return None

        candidates.sort(key=lambda x: x[0])
        return candidates[0][1]

    def send_goal(self, target: Tuple[float, float]):
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('NavigateToPose action server not available yet.')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = self.get_parameter('map_frame').value
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = target[0]
        goal_msg.pose.pose.position.y = target[1]
        goal_msg.pose.pose.position.z = 0.0

        # Keep neutral heading for now
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(
            f'Sending goal to frontier: x={target[0]:.2f}, y={target[1]:.2f}'
        )

        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

        self.goal_active = True
        self.current_goal = target
        self.goal_start_time = self.get_clock().now()

    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f'Goal request failed: {str(e)}')
            self.goal_active = False
            self.current_goal = None
            return

        if goal_handle is None:
            self.get_logger().warn('No goal handle returned by Nav2.')
            self.goal_active = False
            self.current_goal = None
            return

        if not goal_handle.accepted:
            self.get_logger().warn('Goal was rejected by Nav2.')
            if self.current_goal is not None:
                self.blacklisted_frontiers.append(self.current_goal)
            self.goal_active = False
            self.current_goal = None
            return

        self.get_logger().info('Goal accepted by Nav2.')
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        if self.current_goal is None:
            self.goal_active = False
            return

        try:
            result = future.result()
            status = result.status

            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(
                    f'Reached frontier: x={self.current_goal[0]:.2f}, y={self.current_goal[1]:.2f}'
                )
                self.visited_frontiers.append(self.current_goal)

            else:
                self.get_logger().warn(
                    f'Failed to reach frontier: x={self.current_goal[0]:.2f}, '
                    f'y={self.current_goal[1]:.2f}, status={status}'
                )
                self.blacklisted_frontiers.append(self.current_goal)

        except Exception as e:
            self.get_logger().error(f'Error while getting navigation result: {str(e)}')
            self.blacklisted_frontiers.append(self.current_goal)

        self.goal_active = False
        self.current_goal = None
        self.result_future = None
        self.goal_start_time = None

    def control_loop(self):
        if self.goal_active:
            return

        if not self.frontiers:
            self.get_logger().debug('No frontier centroids available.')
            return

        robot_pos = self.get_robot_position()
        if robot_pos is None:
            return

        best_frontier = self.choose_best_frontier(robot_pos)

        if best_frontier is None:
            self.get_logger().info('No valid frontier available to navigate to.')
            return

        self.send_goal(best_frontier)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()