#!/usr/bin/env python3
"""
RANSAC Plane Segmentation Node

Architecture:
    - Subscribe: /points_filtered   (sensor_msgs/PointCloud2)
    - Publish:   /points_obstacles  (sensor_msgs/PointCloud2) - non-plane points only
    - Publish:   /points_plane      (sensor_msgs/PointCloud2) - plane points (floor/wall)

Why two outputs?
    - /points_obstacles: fed into Clustering for object detection
    - /points_plane: useful for visualization and debugging (show detected floor)

Parameters:
    - distance_threshold (float): Max distance from plane to count as inlier (default: 0.02m = 2cm)
    - ransac_iterations (int): How many random samples to try (default: 100)
    - min_inliers (int): Minimum points to consider a valid plane (default: 100)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np


class PlaneSegmentation(Node):

    def __init__(self):
        super().__init__('plane_segmentation')

        self.declare_parameter('distance_threshold', 0.02)
        self.declare_parameter('ransac_iterations', 100)
        self.declare_parameter('min_inliers', 100)
        self.declare_parameter('input_topic', '/points_filtered')
        self.declare_parameter('output_obstacles_topic', '/points_obstacles')
        self.declare_parameter('output_plane_topic', '/points_plane')

        self.dist_thresh = self.get_parameter('distance_threshold').value
        self.max_iter = self.get_parameter('ransac_iterations').value
        self.min_inliers = self.get_parameter('min_inliers').value
        input_topic = self.get_parameter('input_topic').value
        out_obstacles = self.get_parameter('output_obstacles_topic').value
        out_plane = self.get_parameter('output_plane_topic').value

        self.subscription = self.create_subscription(
            PointCloud2, input_topic, self.callback, 10)
        self.pub_obstacles = self.create_publisher(PointCloud2, out_obstacles, 10)
        self.pub_plane = self.create_publisher(PointCloud2, out_plane, 10)

        self.get_logger().info(
            f'Plane Segmentation initialized\n'
            f'  distance_threshold: {self.dist_thresh}m\n'
            f'  ransac_iterations: {self.max_iter}\n'
            f'  Input:  {input_topic}\n'
            f'  Output (obstacles): {out_obstacles}\n'
            f'  Output (plane):     {out_plane}'
        )
        self.count = 0

    def callback(self, msg):
        import time
        t0 = time.time()

        # Step 1: PointCloud2 → NumPy (N, 3)
        points = pc2.read_points_numpy(
            msg, field_names=("x", "y", "z"), skip_nans=True
        ).astype(np.float32)

        if len(points) < 3:
            return

        # Step 2: RANSAC Plane Fitting
        best_inliers_mask = self.ransac_plane(points)

        # Step 3: Split into plane / obstacles
        plane_points = points[best_inliers_mask]
        obstacle_points = points[~best_inliers_mask]

        # Step 4: Publish both outputs
        self.pub_obstacles.publish(
            pc2.create_cloud_xyz32(msg.header, obstacle_points.tolist()))
        self.pub_plane.publish(
            pc2.create_cloud_xyz32(msg.header, plane_points.tolist()))

        elapsed = (time.time() - t0) * 1000
        self.count += 1
        if self.count % 10 == 0:
            self.get_logger().info(
                f'Processed {self.count} | '
                f'Points: {len(points)} → obstacles:{len(obstacle_points)} '
                f'+ plane:{len(plane_points)} | '
                f'Time: {elapsed:.1f}ms'
            )

    def ransac_plane(self, points):
        """
        RANSAC algorithm to find the dominant plane.

        Core idea:
            1. Randomly pick 3 points → define a plane
            2. Count how many other points are close to this plane (inliers)
            3. Repeat max_iter times
            4. Return the plane with the most inliers

        A plane in 3D is defined by: ax + by + cz + d = 0
        where (a, b, c) is the normal vector of the plane.

        Returns:
            np.array of bool: True = inlier (part of plane), False = obstacle
        """
        n = len(points)
        best_mask = np.zeros(n, dtype=bool)
        best_count = 0

        for _ in range(self.max_iter):
            # Step A: Randomly pick 3 points
            idx = np.random.choice(n, 3, replace=False)
            p1, p2, p3 = points[idx[0]], points[idx[1]], points[idx[2]]

            # Step B: Compute plane normal via cross product
            # v1, v2 are two vectors lying in the plane
            v1 = p2 - p1
            v2 = p3 - p1
            normal = np.cross(v1, v2)

            # Skip degenerate case (3 collinear points → zero normal)
            norm_len = np.linalg.norm(normal)
            if norm_len < 1e-6:
                continue
            normal = normal / norm_len  # unit normal

            # Step C: Plane equation: normal · (point - p1) = 0
            # Distance of all points from this plane
            # distance = |normal · (point - p1)|
            dist = np.abs((points - p1) @ normal)  # (N,)

            # Step D: Count inliers
            mask = dist < self.dist_thresh
            count = mask.sum()

            if count > best_count:
                best_count = count
                best_mask = mask

        # Only return plane if it has enough inliers
        if best_count < self.min_inliers:
            return np.zeros(n, dtype=bool)  # no plane found

        return best_mask


def main(args=None):
    rclpy.init(args=args)
    try:
        node = PlaneSegmentation()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
