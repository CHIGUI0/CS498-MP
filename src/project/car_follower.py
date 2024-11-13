#!/usr/bin/env python3

# ================================================================
# File name: gem_gnss_pp_tracker_modified.py                                                                  
# Description: Pure Pursuit controller using road curve equation                                                                
# Author: Hang Cui
# Email: hangcui3@illinois.edu                                                                     
# Date created: 08/02/2021                                                                
# Date last modified: [当前日期]                                                          
# Version: 2.0                                                                   
# Usage: rosrun gem_gnss gem_gnss_pp_tracker_modified.py                                                                      
# Python version: 3.8                                                             
# ================================================================

from __future__ import print_function

# Python Headers
import math
import numpy as np

# ROS Headers
import rospy

# GEM Sensor Headers
from std_msgs.msg import Float32MultiArray
from ackermann_msgs.msg import AckermannDriveStamped


class PurePursuitController(object):

    def __init__(self):

        self.rate = rospy.Rate(80)

        self.look_ahead = 0.4  # 前瞻距离，单位：米
        self.wheelbase = 0.325  # 车辆轴距，单位：米

        # 发布控制指令
        self.ctrl_pub = rospy.Publisher(
            "/vesc/low_level/ackermann_cmd_mux/input/navigation",
            AckermannDriveStamped,
            queue_size=1
        )

        self.drive_msg = AckermannDriveStamped()
        self.drive_msg.header.frame_id = "f1tenth_control"
        self.drive_msg.drive.speed = 1.2  # 车辆速度，单位：米/秒

        # 订阅道路曲线系数
        # 假设道路曲线的一元二次方程系数以 [a, b, c] 的形式发布在 '/road_curve' 话题上
        self.curve_sub = rospy.Subscriber(
            '/road_curve',
            Float32MultiArray,
            self.curve_callback
        )

        # 道路曲线系数，初始化为直线 y = 0
        self.curve_coeffs = [0.0, 0.0, 0.0]  # [a, b, c] for y = a*x^2 + b*x + c

    def curve_callback(self, curve_msg):
        # 更新道路曲线系数
        self.curve_coeffs = curve_msg.data  # [a, b, c]

    def compute_target_point(self):
        """
        基于道路曲线和前瞻距离，计算目标点的相对坐标。
        """

        # 在道路曲线上寻找距离车辆前瞻距离的目标点
        # 由于曲线是二维的，我们需要解方程来找到合适的 x 值

        a, b, c = self.curve_coeffs

        # 假设车辆在 (0, 0)，朝向 x 轴正方向
        # 计算沿曲线距离为 look_ahead 的点

        # 为了简化计算，我们可以在 x 轴上取增量，找到与前瞻距离最接近的点
        # 这里我们可以采用数值方法，如 bisection 或者使用小步长遍历 x

        # 定义函数来计算曲线上两点之间的弧长近似
        def compute_arc_length(x_start, x_end, num=100):
            x = np.linspace(x_start, x_end, num)
            y = a * x ** 2 + b * x + c
            dx = np.diff(x)
            dy = np.diff(y)
            ds = np.hypot(dx, dy)
            return np.sum(ds)

        # 初始化搜索范围和精度
        x_start = 0.0  # 从车辆当前位置开始
        x_end = x_start + self.look_ahead  # 初始猜测
        arc_length = 0.0

        # 通过增量搜索找到满足弧长接近前瞻距离的 x 值
        while arc_length < self.look_ahead:
            x_end += 0.01  # 增加 x_end
            arc_length = compute_arc_length(0.0, x_end)

        # 目标点的 x 坐标为 x_end，y 坐标为对应的曲线值
        target_x = x_end
        target_y = a * target_x ** 2 + b * target_x + c

        return target_x, target_y

    def start_pure_pursuit(self):
        while not rospy.is_shutdown():

            # 计算目标点
            target_x, target_y = self.compute_target_point()

            # 计算坐标系下的偏移量
            # 由于车辆在 (0, 0)，朝向 x 轴正方向，所以无需转换坐标

            # 计算 alpha
            # alpha 是车辆朝向与目标点连线之间的夹角
            alpha = math.atan2(target_y, target_x)

            # 计算转向角度（使用纯追踪控制算法）
            # delta = arctangent(2 * L * sin(alpha) / Ld)
            L = self.wheelbase
            Ld = math.hypot(target_x, target_y)  # 前瞻距离
            steering_angle = math.atan2(2 * L * math.sin(alpha), Ld)

            # 限制转向角在物理范围内
            max_steering_angle = 0.4  # 弧度（约23度）
            steering_angle = np.clip(steering_angle, -max_steering_angle, max_steering_angle)

            # 发布控制指令
            self.drive_msg.header.stamp = rospy.get_rostime()
            self.drive_msg.drive.steering_angle = steering_angle
            self.ctrl_pub.publish(self.drive_msg)

            # 调试信息
            steering_angle_deg = math.degrees(steering_angle)
            print(f"Target point: ({target_x:.3f}, {target_y:.3f})")
            print(f"Steering angle: {steering_angle_deg:.2f} degrees")
            print("\n")

            self.rate.sleep()


def pure_pursuit_control():
    rospy.init_node('pure_pursuit_node', anonymous=True)
    controller = PurePursuitController()

    try:
        controller.start_pure_pursuit()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    pure_pursuit_control()
