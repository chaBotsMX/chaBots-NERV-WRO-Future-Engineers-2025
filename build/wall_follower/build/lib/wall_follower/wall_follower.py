#!/usr/bin/env python3
"""
Wall Follower con visualización de trayectoria en RViz sin TF
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker
import math
import time


class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower_viz')

        # Parámetros de control
        self.WALL_DISTANCE = 0.5
        self.MAX_SPEED = 0.2
        self.P_GAIN = 2.0
        self.WALL_SIDE = -90  # derecha

        # Publicadores y suscriptores
        self.publisher_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.publisher_marker = self.create_publisher(Marker, '/trajectory_marker', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )

        # Lista de puntos para trayectoria
        self.traj_points = []
        self.last_time = time.time()

        self.get_logger().info('Wall Follower con trayectoria iniciado')

    def lidar_callback(self, msg):
        measurements = self.get_wall_measurements(msg)
        if measurements is None:
            self.stop_robot()
            return

        front = measurements['front']
        side = measurements['side']
        diagonal = measurements['diagonal']

        distance_error = self.WALL_DISTANCE - side
        angle_error = (diagonal - side) * 2.0
        angular_velocity = self.P_GAIN * distance_error + angle_error * 0.5

        if front < 0.3:
            linear_velocity = 0.0
            angular_velocity = 0.5 if self.WALL_SIDE < 0 else -0.5
        elif front < 0.6:
            linear_velocity = self.MAX_SPEED * 0.3
        else:
            linear_velocity = self.MAX_SPEED

        self.send_velocity(linear_velocity, angular_velocity)

        # Guardar punto en trayectoria cada 0.2s
        now = time.time()
        if now - self.last_time > 0.2:
            self.traj_points.append(Point(x=len(self.traj_points) * 0.05, y=side, z=0.0))
            self.publish_marker(msg.header.frame_id)
            self.last_time = now

    def get_wall_measurements(self, scan_msg):
        ranges = scan_msg.ranges
        angle_min = scan_msg.angle_min
        angle_increment = scan_msg.angle_increment

        def get_distance_at_angle(desired_angle_deg):
            desired_angle_rad = math.radians(desired_angle_deg)
            index = int((desired_angle_rad - angle_min) / angle_increment)
            if index < 0 or index >= len(ranges):
                return None
            distance = ranges[index]
            if math.isinf(distance) or math.isnan(distance) or distance <= 0.1:
                return None
            valid = [
                d for i, d in enumerate(ranges[max(0, index-2):min(len(ranges), index+3)])
                if not math.isinf(d) and not math.isnan(d) and 0.1 < d < 10.0
            ]
            return min(valid) if valid else None

        front = get_distance_at_angle(0)
        side = get_distance_at_angle(self.WALL_SIDE)
        diagonal = get_distance_at_angle(self.WALL_SIDE / 2)
        if front is None or side is None or diagonal is None:
            return None
        return {'front': front, 'side': side, 'diagonal': diagonal}

    def send_velocity(self, linear, angular):
        msg = Twist()
        msg.linear.x = max(0.0, min(self.MAX_SPEED, linear))
        msg.angular.z = max(-1.0, min(1.0, angular))
        self.publisher_cmd.publish(msg)

    def stop_robot(self):
        msg = Twist()
        self.publisher_cmd.publish(msg)

    def publish_marker(self, frame_id):
        marker = Marker()
        marker.header.frame_id = frame_id  # Usar el mismo del LaserScan
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "trajectory"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.02
        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.points = self.traj_points[-200:]  # Últimos puntos
        self.publisher_marker.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()