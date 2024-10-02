import rospy
from gazebo_msgs.srv import GetModelState, GetModelStateResponse
from gazebo_msgs.msg import ModelState
from ackermann_msgs.msg import AckermannDrive
import numpy as np
from std_msgs.msg import Float32MultiArray
import math
from util import euler_to_quaternion, quaternion_to_euler
import time

class vehicleController():

    def __init__(self):
        # Publisher to publish the control input to the vehicle model
        self.controlPub = rospy.Publisher("/ackermann_cmd", AckermannDrive, queue_size = 1)
        self.prev_vel = 0
        self.L = 1.75 # Wheelbase, can be get from gem_control.py
        self.log_acceleration = True


        # Record the acceleration and Position
        self.acceleration = []
        self.position_x = []
        self.position_y = []

    def getModelState(self):
        # Get the current state of the vehicle
        # Input: None
        # Output: ModelState, the state of the vehicle, contain the
        #   position, orientation, linear velocity, angular velocity
        #   of the vehicle
        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            serviceResponse = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            resp = serviceResponse(model_name='gem')
        except rospy.ServiceException as exc:
            rospy.loginfo("Service did not process request: "+str(exc))
            resp = GetModelStateResponse()
            resp.success = False
        return resp


    # Tasks 1: Read the documentation https://docs.ros.org/en/fuerte/api/gazebo/html/msg/ModelState.html
    #       and extract yaw, velocity, vehicle_position_x, vehicle_position_y
    # Hint: you may use the the helper function(quaternion_to_euler()) we provide to convert from quaternion to euler
    def extract_vehicle_info(self, currentPose):

        ####################### TODO: Your TASK 1 code starts Here #######################
        pos_x, pos_y, vel, yaw = 0, 0, 0, 0

        msg = currentPose

        # Position
        pos_x = msg.pose.position.x
        pos_y = msg.pose.position.y

        # Velocity
        vel_x = msg.twist.linear.x
        vel_y = msg.twist.linear.y
        vel = (vel_x**2 + vel_y**2)**0.5  # Total linear velocity in the XY plane
    
        # Extract orientation and convert to yaw

        _, _, yaw = quaternion_to_euler(
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w)


        ####################### TODO: Your Task 1 code ends Here #######################

        return pos_x, pos_y, vel, yaw # note that yaw is in radian

    # Task 2: Longtitudal Controller
    # Based on all unreached waypoints, and your current vehicle state, decide your velocity
    def longititudal_controller(self, curr_x, curr_y, curr_vel, curr_yaw, future_unreached_waypoints):

        ####################### TODO: Your TASK 2 code starts Here #######################
        target_velocity = 15

        # Get the future point
        target_x, target_y = future_unreached_waypoints[0]
        
        t_yaw = math.atan2(target_y-curr_y, target_x-curr_x)

        # We choose a threshold
        threshold = math.pi/36

        diff = abs(t_yaw-curr_yaw)

        # Turn
        if diff > threshold:
            target_velocity = 8

        v_threshold = 7
        if target_velocity - curr_vel > v_threshold:
            target_velocity = curr_vel + v_threshold - 1


        ####################### TODO: Your TASK 2 code ends Here #######################
        return target_velocity


    # Task 3: Lateral Controller (Pure Pursuit)
    def pure_pursuit_lateral_controller(self, curr_x, curr_y, curr_yaw, target_point, future_unreached_waypoints):

        ####################### TODO: Your TASK 3 code starts Here #######################
        target_steering = 0

        # Kdd = 0.2

        far_target_index = 2

        # Check future points
        if len(future_unreached_waypoints)< far_target_index + 1:
            far_target_index = len(future_unreached_waypoints) - 1

        far_target = future_unreached_waypoints[far_target_index]

        fraction_far = 0.15

        target_x = far_target[0] * fraction_far + target_point[0] * (1-fraction_far)
        target_y = far_target[1] * fraction_far + target_point[1] * (1-fraction_far)


        # target_x, target_y = interplate_point
        
        #  ld
        ld = ((target_x-curr_x)**2 + (target_y-curr_y)**2)**0.5

        t_yaw = math.atan2(target_y-curr_y, target_x-curr_x)

        diff = t_yaw-curr_yaw

        target_steering = math.atan(2*self.L*math.sin(diff)/ld)


        ####################### TODO: Your TASK 3 code starts Here #######################
        return target_steering


    def execute(self, currentPose, target_point, future_unreached_waypoints):
        # Compute the control input to the vehicle according to the
        # current and reference pose of the vehicle
        # Input:
        #   currentPose: ModelState, the current state of the vehicle
        #   target_point: [target_x, target_y]
        #   future_unreached_waypoints: a list of future waypoints[[target_x, target_y]]
        # Output: None

        curr_x, curr_y, curr_vel, curr_yaw = self.extract_vehicle_info(currentPose)

        # Acceleration Profile
        if self.log_acceleration:
            acceleration = (curr_vel- self.prev_vel) * 100 # Since we are running in 100Hz
            self.acceleration.append(acceleration)

        self.position_x.append(curr_x)
        self.position_y.append(curr_y)

        self.prev_vel = curr_vel
        

        target_velocity = self.longititudal_controller(curr_x, curr_y, curr_vel, curr_yaw, future_unreached_waypoints)
        target_steering = self.pure_pursuit_lateral_controller(curr_x, curr_y, curr_yaw, target_point, future_unreached_waypoints)


        #Pack computed velocity and steering angle into Ackermann command
        newAckermannCmd = AckermannDrive()
        newAckermannCmd.speed = target_velocity
        newAckermannCmd.steering_angle = target_steering

        # Publish the computed control input to vehicle model
        self.controlPub.publish(newAckermannCmd)

    def stop(self):
        newAckermannCmd = AckermannDrive()
        newAckermannCmd.speed = 0
        self.controlPub.publish(newAckermannCmd)
