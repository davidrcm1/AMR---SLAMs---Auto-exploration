import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from collections import deque

class FrontierDetector(Node):
    def __init__(self):
        super().__init__('frontier_detector')

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        self.frontier_pub = self.create_publisher(
            Marker,
            '/frontiers',
            10
        )

        self.centroid_pub = self.create_publisher(
            Marker,
            '/frontier_centroids',
            10
        )



        self.get_logger().info('Frontier detector started.')

    def map_callback(self, msg: OccupancyGrid):
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y
        data = msg.data

        frontier_cells = set()

        for y in range(1, height - 1):
            for x in range(1, width - 1):
                idx = y * width + x

                # Frontier cell must be free
                if data[idx] != 0:
                    continue

                # Check 8-connected neighbors for unknown
                has_unknown_neighbor = False
                for ny in range(y - 1, y + 2):
                    for nx in range(x - 1, x + 2):
                        if nx == x and ny == y:
                            continue
                        nidx = ny * width + nx
                        if data[nidx] == -1:
                            has_unknown_neighbor = True
                            break
                    if has_unknown_neighbor:
                        break

                if has_unknown_neighbor:
                    frontier_cells.add((x, y))

        clusters = self.cluster_frontiers(frontier_cells)

        centroids = self.compute_cluster_centroids(
            clusters,
            origin_x,
            origin_y,
            resolution
        )

        frontier_points = []
        for x, y in frontier_cells:
            wx = origin_x + (x + 0.5) * resolution
            wy = origin_y + (y + 0.5) * resolution
            frontier_points.append((wx, wy))

        self.publish_frontier_marker(frontier_points, msg.header.frame_id)
        self.publish_centroids(centroids, msg.header.frame_id)

        self.get_logger().info(
            f'Detected {len(frontier_cells)} frontier cells in {len(centroids)} clusters.'
        )

    def cluster_frontiers(self, frontier_cells):
        visited = set()
        clusters = []

        neighbors = [
            (-1, -1), (0, -1), (1, -1),
            (-1,  0),          (1,  0),
            (-1,  1), (0,  1), (1,  1),
        ]

        for cell in frontier_cells:
            if cell in visited:
                continue

            cluster = []
            queue = deque([cell])
            visited.add(cell)

            while queue:
                cx, cy = queue.popleft()
                cluster.append((cx, cy))

                for dx, dy in neighbors:
                    nx, ny = cx + dx, cy + dy
                    neighbor = (nx, ny)

                    if neighbor in frontier_cells and neighbor not in visited:
                        visited.add(neighbor)
                        queue.append(neighbor)

            clusters.append(cluster)

        return clusters
    
    def compute_cluster_centroids(self, clusters, origin_x, origin_y, resolution, min_cluster_size=10):
        centroids = []

        for cluster in clusters:
            if len(cluster) < min_cluster_size:
                continue

            avg_x = sum(x for x, y in cluster) / len(cluster)
            avg_y = sum(y for x, y in cluster) / len(cluster)

            wx = origin_x + (avg_x + 0.5) * resolution
            wy = origin_y + (avg_y + 0.5) * resolution

            centroids.append((wx, wy, len(cluster)))

        return centroids

    def publish_frontier_marker(self, frontier_points, frame_id):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'frontiers'
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        for x, y in frontier_points:
            p = Point()
            p.x = x
            p.y = y
            p.z = 0.02
            marker.points.append(p)

        self.frontier_pub.publish(marker)

    def publish_centroids(self, centroids, frame_id):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'frontier_centroids'
        marker.id = 1
        marker.type = Marker.POINTS
        marker.action = Marker.ADD

        marker.scale.x = 0.15
        marker.scale.y = 0.15

        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        for x, y, _ in centroids:
            p = Point()
            p.x = x
            p.y = y
            p.z = 0.05
            marker.points.append(p)

        self.centroid_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()