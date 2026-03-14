#!/usr/bin/env python3
"""
Turtle Controller - 자동 Point-to-Point Navigation

아키텍처 패턴:
- Closed-Loop Control: Pose 피드백 기반 제어
- State Machine: ROTATING → MOVING → ARRIVED
- Proportional Control: 오차에 비례한 제어 출력
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math


class TurtleController(Node):
    """거북이 자동 제어 노드"""

    # State Machine 상태 정의
    STATE_ROTATING = 0
    STATE_MOVING = 1
    STATE_ARRIVED = 2

    def __init__(self):
        super().__init__('turtle_controller')

        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        # Parameters
        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        self.declare_parameter('goal_x', 8.0)
        self.declare_parameter('goal_y', 8.0)
        self.declare_parameter('linear_speed', 1.5)
        self.declare_parameter('angular_speed', 1.5)
        self.declare_parameter('distance_tolerance', 0.1)
        self.declare_parameter('angle_tolerance', 0.05)

        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.distance_tolerance = self.get_parameter('distance_tolerance').value
        self.angle_tolerance = self.get_parameter('angle_tolerance').value

        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        # State
        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        self.current_pose = None
        self.state = self.STATE_ROTATING
        self.previous_distance = float('inf')  # Overshoot 감지용

        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        # Subscriber: 현재 위치 피드백
        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        self.pose_subscriber = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )

        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        # Publisher: 제어 명령 출력
        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        self.cmd_publisher = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10
        )

        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        # Control Loop Timer (10Hz)
        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('🐢 Turtle Controller Started!')
        self.get_logger().info(f'   Goal: ({self.goal_x:.2f}, {self.goal_y:.2f})')

    def pose_callback(self, msg):
        """위치 피드백 수신"""
        self.current_pose = msg

    def control_loop(self):
        """메인 제어 루프 (10Hz)"""
        if self.current_pose is None:
            return  # 아직 위치 정보 없음

        # 목표까지의 거리 및 각도 계산
        distance = self.get_distance_to_goal()
        angle_to_goal = self.get_angle_to_goal()
        angle_error = self.normalize_angle(angle_to_goal - self.current_pose.theta)

        # 디버깅 로그 (처음 한 번만)
        if not hasattr(self, '_debug_logged'):
            self.get_logger().info(
                f'🎯 Initial State:\n'
                f'   Current: ({self.current_pose.x:.2f}, {self.current_pose.y:.2f}), theta={math.degrees(self.current_pose.theta):.1f}°\n'
                f'   Goal: ({self.goal_x:.2f}, {self.goal_y:.2f})\n'
                f'   Distance: {distance:.2f}m\n'
                f'   Angle to goal: {math.degrees(angle_to_goal):.1f}°\n'
                f'   Angle error: {math.degrees(angle_error):.1f}°'
            )
            self._debug_logged = True

        # State Machine
        cmd = Twist()

        if self.state == self.STATE_ROTATING:
            # 회전 중
            if abs(angle_error) > self.angle_tolerance:
                # 아직 목표 방향 아님 → 계속 회전
                cmd.angular.z = self.angular_speed if angle_error > 0 else -self.angular_speed
                self.get_logger().info(
                    f'🔄 ROTATING: angle_error={math.degrees(angle_error):.1f}°'
                )
            else:
                # 목표 방향 도달 → MOVING 상태로 전환
                self.state = self.STATE_MOVING
                self.get_logger().info('✅ Rotation complete → MOVING')

        elif self.state == self.STATE_MOVING:
            # Overshoot 감지: 거리가 증가하기 시작하면 멈춤
            if distance > self.previous_distance + 0.05:
                self.state = self.STATE_ARRIVED
                self.get_logger().warn(
                    f'⚠️  Overshoot detected! Stopping. (distance increased from {self.previous_distance:.2f} to {distance:.2f})'
                )
            # 이동 중
            elif distance > self.distance_tolerance:
                # 아직 목표 거리 아님 → 직진 (+ 약간의 각도 보정)
                cmd.linear.x = self.linear_speed

                # 각도 보정: 방향이 많이 틀어졌으면 약간 회전
                if abs(angle_error) > 0.2:  # ~11도
                    cmd.angular.z = 0.5 * (self.angular_speed if angle_error > 0 else -self.angular_speed)
                    self.get_logger().info(
                        f'➡️  MOVING (correcting): distance={distance:.2f}m, angle_error={math.degrees(angle_error):.1f}°'
                    )
                else:
                    self.get_logger().info(
                        f'➡️  MOVING: distance={distance:.2f}m'
                    )

                self.previous_distance = distance
            else:
                # 목표 거리 도달 → ARRIVED 상태로 전환
                self.state = self.STATE_ARRIVED
                self.get_logger().info('🎯 ARRIVED at goal!')

        elif self.state == self.STATE_ARRIVED:
            # 도착 - 정지
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info('🛑 Staying at goal', throttle_duration_sec=2.0)

        # 제어 명령 발행
        self.cmd_publisher.publish(cmd)

    def get_distance_to_goal(self):
        """유클리드 거리 계산"""
        dx = self.goal_x - self.current_pose.x
        dy = self.goal_y - self.current_pose.y
        return math.sqrt(dx**2 + dy**2)

    def get_angle_to_goal(self):
        """목표 방향 각도 계산 (라디안)"""
        dx = self.goal_x - self.current_pose.x
        dy = self.goal_y - self.current_pose.y
        return math.atan2(dy, dx)

    def normalize_angle(self, angle):
        """각도를 [-π, π] 범위로 정규화"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Shutting down...')
    finally:
        # 정지 명령
        cmd = Twist()
        node.cmd_publisher.publish(cmd)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
