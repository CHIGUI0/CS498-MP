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


from scipy.optimize import fsolve

class PurePursuitController(object):

    def __init__(self):
        # Window Size
        self.window_x = 480
        self.window_y = 640

        self.rate = rospy.Rate(80)

        self.look_ahead =  480*0.9 # 前瞻距离，单位：米
        self.wheelbase = 0.325  # 车辆轴距，单位：米

        # 发布控制指令
        self.ctrl_pub = rospy.Publisher(
            "/vesc/low_level/ackermann_cmd_mux/input/navigation",
            AckermannDriveStamped,
            queue_size=1
        )
        
        # 发布target point
        self.target_pub = rospy.Publisher(
            '/lane_detection/target_point',
            Float32MultiArray,
            queue_size=1
        )

        self.drive_msg = AckermannDriveStamped()
        self.drive_msg.header.frame_id = "f1tenth_control"
        self.drive_msg.drive.speed = 1  # 车辆速度，单位：米/秒


        # 订阅道路曲线系数
        # 假设道路曲线的一元二次方程系数以 [a, b, c] 的形式发布在 '/road_curve' 话题上
        self.curve_sub = rospy.Subscriber(
            '/lane_detection/fit_line_coeff',
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

        def fun_y(x):
            a, b, c = self.curve_coeffs
            return a*x**2 + b*x + c 
        
        # Distance to car position
        def distance(x,y):
            return ((x-0.5*self.window_x)**2 + (y-self.window_y)**2)**0.5
        def dis(x,x0,y0,d,a,b,c):
            y = fun_y(x)
            return math.sqrt((x-x0)**2+(y-y0)**2)-d
        
        x0= self.window_x
        y0= self.window_y*0.5
        d = self.look_ahead
        x_sol = fsolve(dis,320,args=(x0,y0,d,a,b,c))[0]
        y_sol = fun_y(x_sol)
        
        # 发布目标点
        print(x_sol)
        print(y_sol)
        self.target_pub.publish(Float32MultiArray(data=[x_sol, y_sol]))
        
        target_x = y_sol - 0.5*self.window_y
        target_y = self.window_x - x_sol+0.4*480/0.9

        return target_x, target_y

    def start_pure_pursuit(self):
        while not rospy.is_shutdown():

            # 计算目标点
            target_x, target_y = self.compute_target_point()

            # 计算坐标系下的偏移量
            # 由于车辆在 (0, 0)，朝向 x 轴正方向，所以无需转换坐标

            # 计算 alpha
            # alpha 是车辆朝向与目标点连线之间的夹角
            alpha = math.atan2(target_x, target_y)

            # 计算转向角度（使用纯追踪控制算法）
            # delta = arctangent(2 * L * sin(alpha) / Ld)


            scale_coef = 0.9/480
            k = 0.6
            L = self.wheelbase
            Ld = math.hypot(target_x, target_y) * scale_coef  # 前瞻距离
            steering_angle = math.atan2(k * 2 * L * math.sin(alpha), Ld )
            angle   = -steering_angle * 1
            # ----------------- tuning this part as needed -----------------

            f_delta = round(np.clip(angle, -0.3, 0.3), 3)

            f_delta_deg = round(np.degrees(f_delta))

            # print("Current index: " + str(self.goal))
            # ct_error = round(np.sin(alpha) * Ld, 3)
            # print("Crosstrack Error: " + str(ct_error))
            # print("Front steering angle: " + str(f_delta_deg) + " degrees")
            # print("\n")

            self.drive_msg.header.stamp = rospy.get_rostime()
            self.drive_msg.drive.steering_angle = f_delta
            self.ctrl_pub.publish(self.drive_msg)
            # 限制转向角在物理范围内
            # max_steering_angle = 0.4  # 弧度（约23度）
            # steering_angle = np.clip(steering_angle, -max_steering_angle, max_steering_angle)

            # # 发布控制指令
            # self.drive_msg.header.stamp = rospy.get_rostime()
            # self.drive_msg.drive.steering_angle = steering_angle
            # self.ctrl_pub.publish(self.drive_msg)

            # # 调试信息
            # steering_angle_deg = math.degrees(steering_angle)
            # print(f"Target point: ({target_x:.3f}, {target_y:.3f})")
            # print(f"Steering angle: {steering_angle_deg:.2f} degrees")
            # print("\n")

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
