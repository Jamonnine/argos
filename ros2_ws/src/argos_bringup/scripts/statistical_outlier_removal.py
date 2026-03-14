#!/usr/bin/env python3
"""
Statistical Outlier Removal Node

Architecture:
    - Subscribe: /points_downsampled (sensor_msgs/PointCloud2)
    - Publish:   /points_filtered    (sensor_msgs/PointCloud2)
    - Single Responsibility: Only noise removal, nothing else

Algorithm (Statistical Outlier Removal):
    1. For each point, find its K nearest neighbors
    2. Compute the mean distance to those K neighbors
    3. Compute global mean and std deviation of all mean distances
    4. If a point's mean distance > (global_mean + std_multiplier * std_dev) → outlier → remove

Parameters:
    - k_neighbors (int): How many neighbors to check per point (default: 20)
    - std_multiplier (float): How strict the threshold is (default: 2.0)
      Lower = more aggressive removal, Higher = more lenient
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from scipy.spatial import cKDTree


class StatisticalOutlierRemoval(Node):

    def __init__(self):
        super().__init__('statistical_outlier_removal')

        self.declare_parameter('k_neighbors', 20)
        self.declare_parameter('std_multiplier', 2.0)
        self.declare_parameter('input_topic', '/points_downsampled')
        self.declare_parameter('output_topic', '/points_filtered')

        self.k = self.get_parameter('k_neighbors').value
        self.std_mult = self.get_parameter('std_multiplier').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.subscription = self.create_subscription(
            PointCloud2, input_topic, self.callback, 10)
        self.publisher = self.create_publisher(
            PointCloud2, output_topic, 10)

        self.get_logger().info(
            f'Statistical Outlier Removal initialized\n'
            f'  k_neighbors: {self.k}\n'
            f'  std_multiplier: {self.std_mult}\n'
            f'  Input:  {input_topic}\n'
            f'  Output: {output_topic}'
        )
        self.count = 0

    def callback(self, msg):
        import time
        t0 = time.time()

        # Step 1: PointCloud2 → NumPy (N, 3)
        points = pc2.read_points_numpy(
            msg, field_names=("x", "y", "z"), skip_nans=True
        ).astype(np.float32)

        if len(points) < self.k + 1:
            self.publisher.publish(msg)
            return

        n = len(points)

        # Step 2: KDTree - find K nearest neighbors efficiently
        # cKDTree is O(N * K * log N) vs O(N^2) for brute force
        # query(points, k=K+1): returns K+1 neighbors (first is self, distance=0)
        tree = cKDTree(points)
        dists, _ = tree.query(points, k=self.k + 1)  # (N, K+1)
        mean_k_dist = dists[:, 1:].mean(axis=1)       # (N,) — exclude self (col 0)

        # Step 4: Global statistics
        global_mean = mean_k_dist.mean()
        global_std = mean_k_dist.std()

        # Step 5: Threshold — keep points within (mean + multiplier * std)
        threshold = global_mean + self.std_mult * global_std
        mask = mean_k_dist <= threshold
        filtered = points[mask]

        # Step 6: Publish filtered cloud
        out_msg = pc2.create_cloud_xyz32(msg.header, filtered.tolist())
        self.publisher.publish(out_msg)

        elapsed = (time.time() - t0) * 1000
        self.count += 1
        if self.count % 10 == 0:
            removed = n - len(filtered)
            self.get_logger().info(
                f'Processed {self.count} | '
                f'Points: {n} → {len(filtered)} '
                f'({removed} removed, {removed/n*100:.1f}%) | '
                f'Time: {elapsed:.1f}ms'
            )


def main(args=None):
    rclpy.init(args=args)
    try:
        node = StatisticalOutlierRemoval()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
