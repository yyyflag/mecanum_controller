#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


class MecanumKinematicsNode(Node):
    def __init__(self):
        super().__init__('mecanum_kinematics_node')

        # 订阅 /cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # 发布到控制器命令话题
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/mecanum_controller/commands',
            10
        )

        # === 请根据你的机器人实际参数修改以下值 ===
        self.wheel_radius = 0.05      # 轮子半径 (单位: 米)
        self.wheel_base = 0.2         # 左右轮中心距离的一半 × 2（即轮距）
        self.track_width = 0.2        # 前后轮中心距离的一半 × 2（即轴距）

        self.get_logger().info("Mecanum kinematics node started.")
        self.get_logger().info(f"Params: r={self.wheel_radius}, L={self.wheel_base}, W={self.track_width}")

    def cmd_vel_callback(self, msg):
        vx = msg.linear.x    # 前进 (+x)
        vy = msg.linear.y    # 左移 (+y) — 注意：ROS 中 y 向左为正
        wz = msg.angular.z   # 逆时针旋转 (+z)

        L = self.wheel_base / 2.0
        W = self.track_width / 2.0
        R = self.wheel_radius

        # 麦轮逆运动学公式（四轮独立驱动）
        # 参考标准麦轮布局（前左、前右、后左、后右）
        v_fl = (vx - vy - (L + W) * wz) / R
        v_fr = (vx + vy + (L + W) * wz) / R
        v_bl = (vx + vy - (L + W) * wz) / R
        v_br = (vx - vy + (L + W) * wz) / R

        # 构造消息
        cmd = Float64MultiArray()
        cmd.data = [v_fl, v_fr, v_bl, v_br]

        self.publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = MecanumKinematicsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
