from __future__ import print_function

# Python Headers
import math
import numpy as np

# ROS Headers
import rospy

# GEM Sensor Headers
from std_msgs.msg import Float32MultiArray
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan

from scipy.optimize import fsolve

class PurePursuitController(object):

    def __init__(self):
        # Window Size
        self.window_x = 480
        self.window_y = 640

        self.rate = rospy.Rate(80)

        self.look_ahead = 480 * 0.9  # 前瞻距离，单位：米
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
        self.curve_sub = rospy.Subscriber(
            '/lane_detection/fit_line_coeff',
            Float32MultiArray,
            self.curve_callback
        )

        # 订阅LIDAR数据
        self.lidar_sub = rospy.Subscriber(
            '/scan',  # 假设LIDAR数据发布在这个话题上
            LaserScan,
            self.lidar_callback
        )

        # 道路曲线系数，初始化为直线 y = 0
        self.curve_coeffs = [0.0, 0.0, 0.0]  # [a, b, c] for y = a*x^2 + b*x + c

        # 最近障碍物距离初始化
        self.closest_obstacle_distance = float('inf')

    def curve_callback(self, curve_msg):
        # 更新道路曲线系数
        self.curve_coeffs = curve_msg.data  # [a, b, c]

    def lidar_callback(self, scan_data):
        # 定义前方扫描的角度范围（以弧度为单位）
        front_angle_min = -math.radians(10)  # -10度
        front_angle_max = math.radians(10)   # 10度

        # 计算感兴趣的索引范围
        start_index = int((front_angle_min - scan_data.angle_min) / scan_data.angle_increment)
        end_index = int((front_angle_max - scan_data.angle_min) / scan_data.angle_increment)
        
        # 确保索引在有效范围内
        start_index = max(0, start_index)
        end_index = min(len(scan_data.ranges), end_index)

        # 在前方角度范围内找到最小距离
        front_ranges = scan_data.ranges[start_index:end_index]
        self.closest_obstacle_distance = min(front_ranges)
        
        if self.closest_obstacle_distance < 0.8:
            self.drive_msg.drive.speed = 0
        else:
            self.drive_msg.drive.speed = 1
        
        rospy.loginfo("Closest obstacle distance in front direction: %.2f meters", self.closest_obstacle_distance)

    def compute_target_point(self):
        """
        基于道路曲线和前瞻距离，计算目标点的相对坐标。
        """

        # 在道路曲线上寻找距离车辆前瞻距离的目标点
        a, b, c = self.curve_coeffs

        def fun_y(x):
            return a * x**2 + b * x + c 
        
        # Distance to car position
        def distance(x, y):
            return ((x - 0.5 * self.window_x)**2 + (y - self.window_y)**2)**0.5
        
        def dis(x, x0, y0, d, a, b, c):
            y = fun_y(x)
            return math.sqrt((x - x0)**2 + (y - y0)**2) - d
        
        x0 = self.window_x
        y0 = self.window_y * 0.5
        d = self.look_ahead
        x_sol = fsolve(dis, 320, args=(x0, y0, d, a, b, c))[0]
        y_sol = fun_y(x_sol)
        
        # 发布目标点
        print(x_sol)
        print(y_sol)
        self.target_pub.publish(Float32MultiArray(data=[x_sol, y_sol]))
        
        target_x = y_sol - 0.5 * self.window_y
        target_y = self.window_x - x_sol + 0.4 * 480 / 0.9

        return target_x, target_y

    def start_pure_pursuit(self):
        while not rospy.is_shutdown():

            # 计算目标点
            target_x, target_y = self.compute_target_point()

            # 计算 alpha
            alpha = math.atan2(target_x, target_y)

            # 计算转向角度（使用纯追踪控制算法）
            scale_coef = 0.9 / 480
            k = 0.6
            L = self.wheelbase
            Ld = math.hypot(target_x, target_y) * scale_coef
            steering_angle = math.atan2(k * 2 * L * math.sin(alpha), Ld)
            angle = -steering_angle * 1
            f_delta = round(np.clip(angle, -0.3, 0.3), 3)

            self.drive_msg.header.stamp = rospy.get_rostime()
            self.drive_msg.drive.steering_angle = f_delta
            self.ctrl_pub.publish(self.drive_msg)

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
