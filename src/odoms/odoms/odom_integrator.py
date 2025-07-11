#!/usr/bin/env python3

'''

'''
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Accel, Twist, TransformStamped
from ament_index_python.packages import get_package_share_directory
from tf2_ros import TransformBroadcaster
import math
import numpy as np
import os
import yaml

def read_yaml_config(file_path):
    with open(file_path, 'r') as file:
        try:
            config = yaml.safe_load(file)
            return config
        except yaml.YAMLError as exc:
            print(f"Error reading YAML file: {exc}")
            return None

class OdomIntegratorNode(Node):
    def __init__(self):
        super().__init__('odom_2nd_integrator_node')
        self.ns = self.get_namespace()
        self.robot_name = self.ns[1:]
        
        # 获取参数
        config_path = get_package_share_directory('mpc')
        config_file = os.path.join(config_path, 'config', 'params.yaml')
        self.config = read_yaml_config(config_file)
        if self.config is not None:
            # 访问具体的参数
            robots = self.config.get('robot', {})
            pose_x = robots[self.robot_name].get('pose_x', 0.0)
            pose_y = robots[self.robot_name].get('pose_y', 0.0)
        else:
            pose_x = 0.0
            pose_y = 0.0
        self.get_logger().info('pose_x: %.3f, pose_y: %.3f' % (pose_x, pose_y))

        # 初始化状态变量
        self.p = np.zeros(3)
        self.p[0] = pose_x
        self.p[1] = pose_y
        self.v = np.zeros(3)
        self.a = np.zeros(3)

        # 限制幅度
        self.v_limit = 0.5
        # self.vx_limit = 0.5
        # self.vy_limit = 0.5
        # self.vz_limit = 0.5
        self.a_limit = 0.3
        # self.ax_limit = 0.3
        # self.ay_limit = 0.3
        # self.az_limit = 0.3
        
        self.current_time = self.get_clock().now()
        self.last_time = self.get_clock().now()
        
        self.twist_msg = Twist()
        self.accel_msg = Accel()
        
        # 一阶 —— 订阅速度话题（默认/cmd_vel）
        self.subscription = self.create_subscription(
            Twist, 'cmd_vel', self.vel_callback, 10)
        
        # 二阶 —— 订阅加速度话题（例如：/input_accel）
        self.sub_accel = self.create_subscription(
            Accel, 'input_accel', self.accel_callback, 10)
        
        # 发布Odometry话题
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        
        # pose发布定时器
        self.timer = self.create_timer(0.01, self.publish_pose)
        
        # TF广播器
        self.t_map_odom = TransformStamped()
        self.t_map_odom.header.stamp = self.get_clock().now().to_msg()
        self.t_map_odom.header.frame_id = 'map'
        self.t_map_odom.child_frame_id = self.ns + '_odom'
        self.t_map_odom.transform.translation.x = 0.0
        self.t_map_odom.transform.translation.y = 0.0
        self.t_map_odom.transform.translation.z = 0.0
        self.t_map_odom.transform.rotation.w = 1.0  # 无旋转
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.get_logger().info("Odom Integrator Node Started")

    def publish_pose(self):
        # 更新
        self.current_time = self.get_clock().now()
        dt = (self.current_time - self.last_time).nanoseconds / 1e9  # 转换为秒
        self.last_time = self.current_time

        # self.get_logger().info('ax: %.3f m/s^2, ay: %.3f m/s^2, az: %.3f m/s^2' % (self.a[0], self.a[1], self.a[2]))
        # self.get_logger().info('vx: %.3f m/s, vy: %.3f m/s, vz: %.3f m/s' % (self.v[0], self.v[1], self.v[2]))
        # 速度积分（加速度 → 速度）
        self.v += self.a * dt
        self.v = np.clip(self.v, -self.v_limit, self.v_limit)
        # 位置积分（速度 → 位置）
        self.p += self.v * dt
        
        # 发布Odometry消息
        odom_msg = Odometry()
        odom_msg.header.stamp = self.current_time.to_msg()
        odom_msg.header.frame_id = self.ns + '_odom'
        odom_msg.child_frame_id = self.ns + '_base_link'
        # 设置位置
        odom_msg.pose.pose.position.x = self.p[0]
        odom_msg.pose.pose.position.y = self.p[1]
        odom_msg.pose.pose.position.z = self.p[2]        
        # 假设无旋转（可根据角速度扩展）
        odom_msg.pose.pose.orientation.w = 1.0
        
        # 设置速度
        odom_msg.twist.twist.linear.x = self.v[0]
        odom_msg.twist.twist.linear.y = self.v[1]
        odom_msg.twist.twist.linear.z = self.v[2]
        
        self.odom_pub.publish(odom_msg)
        
        # 发布TF变换
        # map->odom
        self.t_map_odom.header.stamp = self.current_time.to_msg()
        self.tf_broadcaster.sendTransform(self.t_map_odom)
        
        t_odom_baselink = TransformStamped()
        t_odom_baselink.header.stamp = self.get_clock().now().to_msg()
        t_odom_baselink.header.frame_id = self.ns + '_odom'
        t_odom_baselink.child_frame_id = self.ns + '_base_link'
        t_odom_baselink.transform.translation.x = self.p[0]
        t_odom_baselink.transform.translation.y = self.p[1]
        t_odom_baselink.transform.translation.z = self.p[2]
        t_odom_baselink.transform.rotation.w = 1.0  # 无旋转
        self.tf_broadcaster.sendTransform(t_odom_baselink)
        
    def vel_callback(self, msg):
        # self.get_logger().info('first order integrator.')
        # 更新
        # self.current_time = self.get_clock().now()
        # dt = (self.current_time - self.last_time).nanoseconds / 1e9  # 转换为秒
        # self.last_time = self.current_time
        self.twist_msg = msg
        # 限幅
        original_vel = np.array([self.twist_msg.linear.x, self.twist_msg.linear.y, self.twist_msg.linear.z])
        self.v = np.clip(original_vel, -self.v_limit, self.v_limit)
        self.a = np.zeros(3)
        # 速度积分（加速度 → 速度）
        # self.v += self.a * dt
        # self.v = np.clip(self.v, -self.v_limit, self.v_limit)
        # 位置积分（速度 → 位置）
        # self.p += self.v * dt

    def accel_callback(self, msg):
        # self.get_logger().info('second order integrator.')
        self.accel_msg = msg
        # 限幅
        original_accel = np.array([self.accel_msg.linear.x, self.accel_msg.linear.y, self.accel_msg.linear.z])
        self.a = np.clip(original_accel, -self.a_limit, self.a_limit)

def main(args=None):
    rclpy.init(args=args)
    node = OdomIntegratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()