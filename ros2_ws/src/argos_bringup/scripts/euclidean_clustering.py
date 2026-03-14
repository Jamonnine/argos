#!/usr/bin/env python3
"""
Euclidean Clustering Node

Architecture:
    - Subscribe: /points_obstacles  (sensor_msgs/PointCloud2)
    - Publish:   /cluster_markers   (visualization_msgs/MarkerArray) - colored clusters in RViz
    - Publish:   /cluster_count     (std_msgs/Int32) - number of detected objects

Algorithm (Euclidean Clustering):
    Group points into clusters where each point is within cluster_tolerance
    distance of at least one other point in the same cluster.
    This is essentially a 3D connected-components problem.

Parameters:
    - cluster_tolerance (float): Max distance between points in same cluster (default: 0.1m)
    - min_cluster_size (int): Minimum points to count as a valid cluster (default: 5)
    - max_cluster_size (int): Maximum points in one cluster (default: 5000)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Int32, ColorRGBA
from geometry_msgs.msg import Point
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from scipy.spatial import cKDTree


# Distinct colors for different clusters (RGB)
CLUSTER_COLORS = [
    (1.0, 0.0, 0.0),  # Red
    (0.0, 1.0, 0.0),  # Green
    (0.0, 0.0, 1.0),  # Blue
    (1.0, 1.0, 0.0),  # Yellow
    (1.0, 0.0, 1.0),  # Magenta
    (0.0, 1.0, 1.0),  # Cyan
    (1.0, 0.5, 0.0),  # Orange
    (0.5, 0.0, 1.0),  # Purple
]


class EuclideanClustering(Node):

    def __init__(self):
        super().__init__('euclidean_clustering')

        self.declare_parameter('cluster_tolerance', 0.1)
        self.declare_parameter('min_cluster_size', 5)
        self.declare_parameter('max_cluster_size', 5000)
        self.declare_parameter('input_topic', '/points_obstacles')

        self.tolerance = self.get_parameter('cluster_tolerance').value
        self.min_size = self.get_parameter('min_cluster_size').value
        self.max_size = self.get_parameter('max_cluster_size').value
        input_topic = self.get_parameter('input_topic').value

        self.subscription = self.create_subscription(
            PointCloud2, input_topic, self.callback, 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/cluster_markers', 10)
        self.pub_count = self.create_publisher(Int32, '/cluster_count', 10)

        self.get_logger().info(
            f'Euclidean Clustering initialized\n'
            f'  cluster_tolerance: {self.tolerance}m\n'
            f'  min_cluster_size: {self.min_size}\n'
            f'  max_cluster_size: {self.max_size}\n'
            f'  Input: {input_topic}'
        )
        self.count = 0

    def callback(self, msg):
        import time
        t0 = time.time()

        # Step 1: PointCloud2 → NumPy
        points = pc2.read_points_numpy(
            msg, field_names=("x", "y", "z"), skip_nans=True
        ).astype(np.float32)

        if len(points) < self.min_size:
            # No obstacles — publish empty marker array to clear RViz
            self.pub_markers.publish(MarkerArray())
            count_msg = Int32()
            count_msg.data = 0
            self.pub_count.publish(count_msg)
            return

        # Step 2: Euclidean Clustering using KDTree BFS
        clusters = self.cluster_points(points)

        # Step 3: Build visualization markers (bounding sphere per cluster)
        markers = self.build_markers(clusters, msg.header)
        self.pub_markers.publish(markers)

        # Step 4: Publish cluster count
        count_msg = Int32()
        count_msg.data = len(clusters)
        self.pub_count.publish(count_msg)

        elapsed = (time.time() - t0) * 1000
        self.count += 1
        if self.count % 10 == 0:
            self.get_logger().info(
                f'Processed {self.count} | '
                f'Points: {len(points)} → {len(clusters)} clusters | '
                f'Time: {elapsed:.1f}ms'
            )

    def cluster_points(self, points):
        """
        Euclidean Clustering via KDTree + BFS (Breadth-First Search).

        Algorithm:
            1. Build KDTree from all points
            2. For each unvisited point:
               a. Find all neighbors within cluster_tolerance
               b. Add them to current cluster, mark as visited
               c. Recursively find neighbors of neighbors (BFS)
            3. If cluster size is in valid range → keep it

        This is the same as "connected components" but in 3D space
        with distance-based connectivity.
        """
        n = len(points)
        visited = np.zeros(n, dtype=bool)
        tree = cKDTree(points)
        clusters = []

        for i in range(n):
            if visited[i]:
                continue

            # BFS from point i
            cluster = []
            queue = [i]
            visited[i] = True

            while queue:
                current = queue.pop(0)
                cluster.append(current)

                # Find all neighbors within tolerance
                neighbors = tree.query_ball_point(points[current], self.tolerance)
                for nb in neighbors:
                    if not visited[nb]:
                        visited[nb] = True
                        queue.append(nb)

            # Filter by size
            if self.min_size <= len(cluster) <= self.max_size:
                clusters.append(points[cluster])

        return clusters

    def build_markers(self, clusters, header):
        """
        Build RViz MarkerArray: Axis-Aligned Bounding Box (AABB) per cluster.

        AABB calculation:
            - min_pt: minimum x, y, z across all points in cluster
            - max_pt: maximum x, y, z across all points in cluster
            - center: midpoint between min and max
            - size: max - min (width, depth, height of the box)

        Two markers per cluster:
            - CUBE (wireframe-style, transparent): shows bounding box volume
            - TEXT_VIEW_FACING: shows cluster ID and point count above the box
        """
        marker_array = MarkerArray()
        marker_id = 0

        for i, cluster in enumerate(clusters):
            color = CLUSTER_COLORS[i % len(CLUSTER_COLORS)]

            # Compute AABB
            min_pt = cluster.min(axis=0)
            max_pt = cluster.max(axis=0)
            center = (min_pt + max_pt) / 2.0
            size = max_pt - min_pt

            # Ensure minimum visible box size (avoid zero-size)
            size = np.maximum(size, 0.05)

            # Marker 1: Bounding Box (CUBE, semi-transparent)
            box = Marker()
            box.header = header
            box.ns = 'bounding_boxes'
            box.id = marker_id
            marker_id += 1
            box.type = Marker.CUBE
            box.action = Marker.ADD
            box.pose.position.x = float(center[0])
            box.pose.position.y = float(center[1])
            box.pose.position.z = float(center[2])
            box.pose.orientation.w = 1.0
            box.scale.x = float(size[0])
            box.scale.y = float(size[1])
            box.scale.z = float(size[2])
            box.color.r = color[0]
            box.color.g = color[1]
            box.color.b = color[2]
            box.color.a = 0.3  # transparent so we can see inside
            box.lifetime.sec = 1

            # Marker 2: Label (TEXT above the box)
            label = Marker()
            label.header = header
            label.ns = 'labels'
            label.id = marker_id
            marker_id += 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = float(center[0])
            label.pose.position.y = float(center[1])
            label.pose.position.z = float(max_pt[2]) + 0.1  # above the box
            label.pose.orientation.w = 1.0
            label.scale.z = 0.1  # text height
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = f'Obj {i} ({len(cluster)}pts)\n{size[0]:.2f}x{size[1]:.2f}x{size[2]:.2f}m'
            label.lifetime.sec = 1

            marker_array.markers.append(box)
            marker_array.markers.append(label)

        return marker_array


def main(args=None):
    rclpy.init(args=args)
    try:
        node = EuclideanClustering()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
