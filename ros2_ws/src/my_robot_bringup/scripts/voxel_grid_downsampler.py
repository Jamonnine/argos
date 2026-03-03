#!/usr/bin/env python3
"""
Voxel Grid Downsampler Node

Purpose:
    Reduces the number of points in a Point Cloud using Voxel Grid filtering.
    This improves processing speed for downstream algorithms while preserving
    overall structure.

Architecture:
    - Subscribe: /camera/points (sensor_msgs/PointCloud2)
    - Publish: /points_downsampled (sensor_msgs/PointCloud2)
    - Single Responsibility: Only downsampling, nothing else
    - Configurable: voxel_size parameter for different scenarios

Performance:
    - Input: ~300,000 points
    - Output: ~20,000 points (15x reduction)
    - Processing time: ~0.01-0.02 seconds
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np


class VoxelGridDownsampler(Node):
    """
    Point Cloud Downsampling using Voxel Grid Filter

    Design Philosophy:
        - Do one thing well (Unix Philosophy)
        - Stateless processing (no memory between callbacks)
        - Fast and deterministic (same input → same output)
    """

    def __init__(self):
        super().__init__('voxel_grid_downsampler')

        # ===== Parameters =====
        # Voxel size: Trade-off between detail and performance
        # - Smaller (0.005m): More detail, slower, more points
        # - Larger (0.05m): Less detail, faster, fewer points
        # - Default (0.01m): Good balance for most applications
        self.declare_parameter('voxel_size', 0.01)
        self.voxel_size = self.get_parameter('voxel_size').value

        # Input/output topic names (configurable via remapping)
        self.declare_parameter('input_topic', '/camera/points')
        self.declare_parameter('output_topic', '/points_downsampled')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        # ===== ROS 2 Communication =====
        # Subscriber: Raw Point Cloud
        self.subscription = self.create_subscription(
            PointCloud2,
            input_topic,
            self.pointcloud_callback,
            10  # QoS: Keep last 10 messages
        )

        # Publisher: Downsampled Point Cloud
        self.publisher = self.create_publisher(
            PointCloud2,
            output_topic,
            10
        )

        # ===== Logging =====
        self.get_logger().info(
            f'Voxel Grid Downsampler initialized\n'
            f'  Voxel size: {self.voxel_size}m\n'
            f'  Input topic: {input_topic}\n'
            f'  Output topic: {output_topic}'
        )

        # Statistics for monitoring
        self.processed_count = 0

    def pointcloud_callback(self, msg):
        """
        Main processing callback

        Flow:
            1. Convert ROS PointCloud2 → NumPy array
            2. Apply Voxel Grid filtering
            3. Convert back to PointCloud2
            4. Publish result

        Why this order?
            - NumPy operations are vectorized (fast)
            - PCL would require C++ bindings (complex)
            - This pure-Python approach is good enough for learning
        """
        import time
        start_time = time.time()

        # Step 1: ROS PointCloud2 → NumPy array (vectorized, no Python loop)
        # read_points_numpy internally converts binary PointCloud2 data to
        # an unstructured numpy array of shape (N, 3) — no Python loop needed
        points = pc2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)

        if len(points) == 0:
            self.get_logger().warn('Received empty point cloud')
            return

        points = points.astype(np.float32)
        original_count = len(points)

        # Step 2: Voxel Grid Filtering (Core Algorithm)
        downsampled_points = self.voxel_grid_filter(points, self.voxel_size)
        downsampled_count = len(downsampled_points)

        # Step 3: NumPy → PointCloud2
        # Create output message with same header (preserve timestamp and frame)
        downsampled_msg = pc2.create_cloud_xyz32(msg.header, downsampled_points.tolist())

        # Step 4: Publish
        self.publisher.publish(downsampled_msg)

        # Performance monitoring
        elapsed_time = time.time() - start_time
        self.processed_count += 1

        if self.processed_count % 10 == 0:  # Log every 10 frames
            reduction_ratio = original_count / downsampled_count if downsampled_count > 0 else 0
            self.get_logger().info(
                f'Processed {self.processed_count} clouds | '
                f'Points: {original_count} → {downsampled_count} '
                f'({reduction_ratio:.1f}x reduction) | '
                f'Time: {elapsed_time*1000:.1f}ms'
            )

    def voxel_grid_filter(self, points, voxel_size):
        """
        Voxel Grid Filtering Algorithm (NumPy Vectorized)

        Algorithm:
            1. Calculate 3D voxel indices for all points at once (vectorized)
            2. Convert 3D indices to 1D flat index (morton-like encoding)
            3. np.unique groups and sorts in one C-level operation
            4. np.add.at accumulates XYZ sums per voxel (no Python loop)
            5. Divide by counts to get centroids

        Performance:
            - Old (Python for loop): ~200-400ms per frame, drops messages
            - New (NumPy vectorized): ~5-15ms per frame, keeps up with camera

        Args:
            points (np.array): Nx3 array of (x, y, z) coordinates
            voxel_size (float): Size of each voxel cube in meters

        Returns:
            np.array: Mx3 array of downsampled points (M < N)
        """
        # Step 1: 3D voxel indices - vectorized for all N points at once
        # np.floor(points / voxel_size) maps each coordinate to voxel grid
        # Example: point (0.123, 0.456, 0.789), voxel_size=0.1
        #   → (floor(1.23), floor(4.56), floor(7.89)) = (1, 4, 7)
        voxel_indices = np.floor(points / voxel_size).astype(np.int32)

        # Step 2: Shift indices to start from 0 (required for flat index math)
        # min_idx: minimum voxel coordinate in each axis
        # shifted: all indices now start from 0 in each axis
        min_idx = voxel_indices.min(axis=0)
        shifted = voxel_indices - min_idx

        # Step 3: Convert 3D index → 1D flat index
        # This is like computing a row-major array offset:
        #   flat = x * (max_y * max_z) + y * max_z + z
        # Why? np.unique works on 1D arrays efficiently.
        # Two points in the same voxel will have the same flat index.
        max_shifted = shifted.max(axis=0) + 1
        flat_indices = (
            shifted[:, 0] * (max_shifted[1] * max_shifted[2]) +
            shifted[:, 1] * max_shifted[2] +
            shifted[:, 2]
        )

        # Step 4: np.unique finds all unique voxels and groups points
        # inverse: for each point, which unique voxel does it belong to?
        # counts: how many points are in each unique voxel?
        # This replaces the Python for loop + dictionary entirely.
        unique_voxels, inverse, counts = np.unique(
            flat_indices, return_inverse=True, return_counts=True
        )

        # Step 5: Accumulate XYZ sums per voxel using np.add.at
        # np.add.at(dst, indices, src): for each i, dst[indices[i]] += src[i]
        # This is an unbuffered operation - handles repeated indices correctly.
        downsampled = np.zeros((len(unique_voxels), 3), dtype=np.float32)
        np.add.at(downsampled, inverse, points)

        # Step 6: Divide by count to get centroid
        downsampled /= counts[:, np.newaxis]

        return downsampled


def main(args=None):
    """
    Main entry point

    Why separate main()?
        - Proper initialization/cleanup
        - Exception handling
        - Multiple node instances possible
    """
    rclpy.init(args=args)

    try:
        node = VoxelGridDownsampler()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
