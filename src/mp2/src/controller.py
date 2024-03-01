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
        pose = currentPose.pose
        twistLinear = currentPose.twist.linear
        euler_state = quaternion_to_euler(pose.orientation.x, 
                                          pose.orientation.y, 
                                          pose.orientation.z, 
                                          pose.orientation.w)
        
        pos_x = pose.position.x
        pos_y = pose.position.y
        vel = math.sqrt(twistLinear.x**2 + twistLinear.y**2 + twistLinear.z**2)
        yaw = euler_state[2]
        ####################### TODO: Your Task 1 code ends Here #######################

        return pos_x, pos_y, vel, yaw # note that yaw is in radian

    # Task 2: Longtitudal Controller
    # Based on all unreached waypoints, and your current vehicle state, decide your velocity
    def longititudal_controller(self, curr_x, curr_y, curr_vel, curr_yaw, future_unreached_waypoints):
        # with open("text.txt", "a") as f:
        #     f.write(f"{curr_x}, {curr_y}\n")
        # with open("future.txt", "a") as f:
        #     f.write(str(future_unreached_waypoints) + "\n")
        ####################### TODO: Your TASK 2 code starts Here #######################
        # print(future_unreached_waypoints)
        maximum_acceleration = 5
        try:

            point1X, point1Y = [curr_x, curr_y]
            point2X, point2Y = future_unreached_waypoints[0]
            point3X, point3Y = future_unreached_waypoints[1]
            dist = np.linalg.norm([point2X-point1X, point2Y-point1Y])
            if dist < 5:
                point2X = (point2X + point3X) / 2
                point2Y = (point2Y + point3Y) / 2

            new1X = point1X - point2X
            new1Y = point1Y - point2Y
            new3X = point3X - point2X
            new3Y = point3Y - point2Y

            norm1X = new1X / np.linalg.norm([new1X, new1Y])
            norm1Y = new1Y / np.linalg.norm([new1X, new1Y])
            norm3X = new3X / np.linalg.norm([new3X, new3Y])
            norm3Y = new3Y / np.linalg.norm([new3X, new3Y])

            curve_val = abs(norm1X * norm3Y - norm1Y * norm3X)
            expect_velocity = 12 - 4 * curve_val / 0.5
            # expect_velocity = 12
        except:
            expect_velocity = 12
        
        if expect_velocity > curr_vel:
            target_velocity = min(curr_vel+maximum_acceleration, expect_velocity)
        else:
            target_velocity = max(curr_vel-maximum_acceleration, expect_velocity)

        # self.prev_vel = target_velocity

        ####################### TODO: Your TASK 2 code ends Here #######################
        return target_velocity


    # Task 3: Lateral Controller (Pure Pursuit)
    def pure_pursuit_lateral_controller(self, curr_x, curr_y, curr_yaw, target_point, future_unreached_waypoints):

        ####################### TODO: Your TASK 3 code starts Here #######################
        # target_steering = 0
        setting = 2
        targetX, targetY = target_point
        dist = np.linalg.norm([targetX-curr_x, targetY-curr_y])
        if setting != 1:
            try:
                look_dist = 10
                if dist < look_dist:
                    rest_dist = look_dist - dist
                    nextX = future_unreached_waypoints[1][0] - targetX
                    nextY = future_unreached_waypoints[1][1] - targetY
                    checkpoint_dist = np.linalg.norm([nextX, nextY])
                    targetX += nextX *rest_dist / checkpoint_dist
                    targetY += nextY *rest_dist / checkpoint_dist
            except:
                pass
        target_yaw = math.atan2((targetY-curr_y),(targetX-curr_x))
        target_steering = math.atan(2 * self.L * math.sin(target_yaw - curr_yaw) / dist)

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
            self.prev_vel = curr_vel
        # with open("acc.txt", "a") as f:
        #     f.write(str(acceleration) + "\n")
        # print(acceleration)



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
