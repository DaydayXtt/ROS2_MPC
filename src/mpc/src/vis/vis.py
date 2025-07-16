#!/usr/bin/env python3
"""
    画积分器的期望轨迹、当前段期望轨迹等    
"""
import numpy as np

import rclpy
import rclpy.duration
from rclpy.node import Node
from rclpy.clock import Clock
import rclpy.qos
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, PoseArray
from builtin_interfaces.msg import Duration

class Visualizer(Node):
    def __init__(self):
        super().__init__("visualizer")
        self.colors = np.array([[149, 165, 129], 
                                [94, 138, 125], 
                                [221, 222, 118], 
                                [219, 63, 87], 
                                [239, 123, 144], 
                                [74, 24, 0], 
                                [240, 24, 120], 
                                [240, 96, 144], 
                                [240, 168, 120], 
                                [99, 194, 184]]) / 255.0
        self.desire_traj_msg = Path()
        self.desire_traj_current_msg = Path()
        self.desire_traj_suber_ = self.create_subscription(
            Path,
            "desire_trajectory",
            self.desire_trajectory_callback,
            10
        )
        self.desire_traj_current_suber_ = self.create_subscription(
            Path,
            "desire_trajectory_current",
            self.desire_trajectory_current_callback,
            10
        )
        self.solved_traj_msg = Path()
        self.solved_traj_suber_ = self.create_subscription(
            Path,
            "solved_trajectory",
            self.solved_trajectory_callback,
            10
        )

        self.obstacles_msg = PoseArray()
        self.obstacles_suber_ = self.create_subscription(
            PoseArray,
            "obstacles",
            self.obstacles_callback,
            10
        )

        marker_qos = rclpy.qos.QoSProfile(
            depth=1,
            # history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        self.marker_array_puber_ = self.create_publisher(
            MarkerArray, "visualizer", marker_qos
        )
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.odom_msg = Odometry()
        self.odom_suber_ = self.create_subscription(
            Odometry,
            "odom",
            self.odom_callback,
            10
        )
        self.odom_traj = []
        self.odom_traj_timer = self.create_timer(0.05, self.odom_traj_timer_callback)

    def odom_traj_timer_callback(self):
        pass
        
    def timer_callback(self):
        # self.get_logger().info('Drawing...')
        marker_array = MarkerArray()
        marker = Marker()
        marker.action = Marker.DELETEALL
        marker.header.stamp = Clock().now().to_msg()
        marker.header.frame_id = "map"
        marker_array.markers.append(marker)

        # 期望轨迹
        self.draw_desire_traj(marker_array, self.desire_traj_msg, "desire") 
        self.draw_desire_current_traj(marker_array, self.desire_traj_current_msg, "desire_current") 
        self.draw_solved_traj(marker_array, self.solved_traj_msg, "solved") 
        self.draw_obs(marker_array, self.obstacles_msg, "obstacles") 
        self.draw_odom_traj(marker_array, self.odom_msg, "odom") 
        
        self.marker_array_puber_.publish(marker_array)    
        
    def desire_trajectory_callback(self, msg):
        self.desire_traj_msg = msg
    def desire_trajectory_current_callback(self, msg):
        self.desire_traj_current_msg = msg
    def solved_trajectory_callback(self, msg):
        self.solved_traj_msg = msg
    def obstacles_callback(self, msg):
        # self.get_logger().info('Obstacle message reveived...')
        self.obstacles_msg = msg
        # self.get_logger().info('Size of obstacles: {}.'.format(len(self.obstacles_msg.poses)))
    def odom_callback(self, msg):
        self.odom_msg = msg
        
    def draw_desire_traj(self, marker_array, desire_traj_msg, ns):
        if desire_traj_msg is None:
            return
        header = desire_traj_msg.header
        poses = desire_traj_msg.poses
        for i in range(0, int(len(poses) / 2) - 1, 12):
            marker = Marker()
            marker.header = header
            marker.ns = ns
            marker.id = i // 12
            marker.type = Marker.LINE_LIST
            marker.action = Marker.ADD
            marker.pose.position.x = 0.0
            marker.pose.position.y = 0.0
            marker.pose.position.z = 0.0
            a = poses[2 * i].pose.position
            b = poses[(2 * i + 11)%len(poses)].pose.position
            marker.points = [a, b]
            marker.scale.x = 0.05
            marker.color.r = self.colors[0][0]
            marker.color.g = self.colors[0][1]
            marker.color.b = self.colors[0][2]
            marker.color.a = 1.0
            # marker.lifetime = Duration(sec=3, nanosec=0)
            marker.frame_locked = True
            marker_array.markers.append(marker)

    def draw_desire_current_traj(self, marker_array, desire_traj_current_msg, ns):
        if desire_traj_current_msg is None:
            return
        header = desire_traj_current_msg.header
        poses = desire_traj_current_msg.poses
        for i in range(len(poses) - 1):
            marker = Marker()
            marker.header = header
            marker.ns = ns
            marker.id = i
            # marker.type = Marker.LINE_STRIP
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = poses[i].pose.position.x
            marker.pose.position.y = poses[i].pose.position.y
            marker.pose.position.z = poses[i].pose.position.z
            # a = poses[i].pose.position
            # b = poses[i + 1].pose.position
            # marker.points = [a, b]
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.r = self.colors[1][0]
            marker.color.g = self.colors[1][1]
            marker.color.b = self.colors[1][2]
            marker.color.a = 1.0
            # marker.lifetime = Duration(sec=3, nanosec=0)
            marker.frame_locked = True
            marker_array.markers.append(marker)

    def draw_solved_traj(self, marker_array, solved_traj_msg, ns):
        if solved_traj_msg is None:
            return
        header = solved_traj_msg.header
        poses = solved_traj_msg.poses
        for i in range(len(poses) - 1):
            marker = Marker()
            marker.header = header
            marker.ns = ns
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.position.x = 0.0
            marker.pose.position.y = 0.0
            marker.pose.position.z = 0.0
            a = poses[i].pose.position
            b = poses[i + 1].pose.position
            marker.points = [a, b]
            marker.scale.x = 0.1
            marker.color.r = self.colors[3][0]
            marker.color.g = self.colors[3][1]
            marker.color.b = self.colors[3][2]
            marker.color.a = 1.0
            # marker.lifetime = Duration(sec=3, nanosec=0)
            marker.frame_locked = True
            marker_array.markers.append(marker)

    def draw_obs(self, marker_array, obstacles_msg, ns):
        if obstacles_msg is None:
            return
        header = obstacles_msg.header
        poses = obstacles_msg.poses
        # 打印障碍物数量
        self.get_logger().info('Size of obstacles: {}.'.format(len(poses)))


        for i in range(len(poses)):
            marker = Marker()
            marker.header = header
            marker.ns = ns
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = poses[i].position.x
            marker.pose.position.y = poses[i].position.y
            marker.pose.position.z = poses[i].position.z
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 1.5
            marker.color.r = self.colors[4][0]
            marker.color.g = self.colors[4][1]
            marker.color.b = self.colors[4][2]
            marker.color.a = 1.0
            # marker.lifetime = Duration(sec=3, nanosec=0)
            marker.frame_locked = True
            marker_array.markers.append(marker)


    def draw_odom_traj(self, marker_array, odom_msg, ns):
        if odom_msg is None:
            return
        point = Point()
        point.x = odom_msg.pose.pose.position.x
        point.y = odom_msg.pose.pose.position.y
        point.z = odom_msg.pose.pose.position.z
        self.odom_traj.append(point)
        if len(self.odom_traj) > 300:
            self.odom_traj.pop(0)
        header = odom_msg.header
        for i in range(len(self.odom_traj) - 1):
            marker = Marker()
            marker.header = header
            marker.ns = ns
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.position.x = 0.0
            marker.pose.position.y = 0.0
            marker.pose.position.z = 0.0
            a = self.odom_traj[i]
            b = self.odom_traj[i + 1]
            marker.points = [a, b]
            marker.scale.x = 0.05
            marker.color.r = self.colors[2][0]
            marker.color.g = self.colors[2][1]
            marker.color.b = self.colors[2][2]
            marker.color.a = 1.0
            # marker.lifetime = Duration(sec=3, nanosec=0)
            marker.frame_locked = True
            marker_array.markers.append(marker)


def main(args=None):
    rclpy.init(args=args)

    visualizer = Visualizer()

    rclpy.spin(visualizer)

    visualizer.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()