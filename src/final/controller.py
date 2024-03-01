import rospy
from gazebo_msgs.srv import GetModelState, GetModelStateResponse
from gazebo_msgs.msg import ModelState
from ackermann_msgs.msg import AckermannDrive
import numpy as np
from scipy.integrate import ode
from std_msgs.msg import Float32MultiArray, String
from util import euler_to_quaternion, quaternion_to_euler
import math
from constant import yaw_range

COMMANDs = {
    "HARDLEFT": 2,   
    "LEFT": 1,
    "STRAIGHT": 0,
    "RIGHT": -1,
    "HARDRIGHT": -2
}


def func1(t, vars, vr, delta):
    curr_x = vars[0]
    curr_y = vars[1]
    curr_theta = vars[2]

    dx = vr * np.cos(curr_theta)
    dy = vr * np.sin(curr_theta)
    dtheta = delta
    return [dx, dy, dtheta]


class bicycleModel:
    def __init__(self, velocity=10, deceleration=0):
        self.waypointSub = rospy.Subscriber(
            "/gem/waypoint", ModelState, self.__waypointHandler, queue_size=1
        )
        self.waypointPub = rospy.Publisher("/gem/waypoint", ModelState, queue_size=1)
        # self.modelStatePub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=1)
        self.controlPub = rospy.Publisher(
            "/ackermann_cmd", AckermannDrive, queue_size=1
        )
        self.decisionMakerSub = rospy.Subscriber(
            "/decision_maker/data", String, self.__decisionHandler, queue_size=1
        )
        self.decisionInitSub = rospy.Subscriber(
            "/decision_maker/init_data", String, self.__initHandler, queue_size=1
        )

        self.state = "STRAIGHT"
        self.vel_state = "BRAKE"

        init_state = self.getModelState()
        self.laneList = []
        self.v_0 = velocity
        self.v_1 = self.v_0
        self.deceleration = deceleration
        self.stopped = False
        self.x = init_state.pose.position.x
        self.y = 0

    def getModelState(self):
        rospy.wait_for_service("/gazebo/get_model_state")
        try:
            serviceResponse = rospy.ServiceProxy(
                "/gazebo/get_model_state", GetModelState
            )
            resp = serviceResponse(model_name="gem")
        except rospy.ServiceException as exc:
            rospy.loginfo("Service did not process request: " + str(exc))
            resp = GetModelStateResponse()
            resp.success = False
        return resp

    def extract_vehicle_info(self, currentPose):
        ####################### TODO: Your TASK 1 code starts Here #######################
        pose = currentPose.pose
        # print(currentPose)
        twistLinear = currentPose.twist.linear
        euler_state = quaternion_to_euler(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )

        pos_x = pose.position.x
        pos_y = pose.position.y
        vel = math.sqrt(twistLinear.x**2 + twistLinear.y**2 + twistLinear.z**2)
        yaw = euler_state[2]
        ####################### TODO: Your Task 1 code ends Here #######################

        return pos_x, pos_y, vel, yaw, currentPose.twist.angular.z  # note that yaw is in radian

    def longititudal_controller(self, curr_x, curr_y, curr_vel, curr_yaw, command):
        ####################### TODO: Modify the speed #######################
        maximum_acceleration = 5
        if command == "BRAKE":
            expect_velocity = 0
        elif abs(COMMANDs[command]) >= 2:
            # the car is hard turning
            expect_velocity = 3
        else:
            expect_velocity = 6

        if expect_velocity > curr_vel:
            target_velocity = min(curr_vel + maximum_acceleration, expect_velocity)
        else:
            target_velocity = max(curr_vel - maximum_acceleration, expect_velocity)

        return target_velocity

    def pure_pursuit_lateral_controller(self, curr_x, curr_y, curr_yaw, command):
        ####################### TODO: Modify the code #######################
        target_steering = COMMANDs[command] * 0.25
        if command == "BRAKE":
            target_steering = 0
        return target_steering

    def get_data_from_state(self, curr_steering):
        # print(self.vel_state, self.state)
        # print(self.state, self.vel_state)
        vel = 0
        acc = 0.2
        angle = 0
        if self.state in ["LEFT", "RIGHT"]:
            angle = COMMANDs[self.state] * 0.3
        if self.state in ["HARDLEFT", "HARDRIGHT"]:
            angle = COMMANDs[self.state] * 0.3
        if self.state == "STRAIGHT":
            angle = 0
        if self.vel_state == "ACCEL":
            vel = 1.5
            acc = 0.35
        if self.vel_state == "BRAKE":
            vel = 0
            acc = 0.2
        # if abs(angle - curr_steering) < yaw_range:
        #     vel = 1.5
        #     acc = 0.35
        # else:
        #     vel = 0.5
        #     acc = 0.2

        return vel, angle, acc

    def setModelState(self, currState, vehicle_state="run"):
        curr_x, curr_y, curr_vel, curr_yaw, curr_steering = self.extract_vehicle_info(currState)

        target_velocity, target_steering, acceleration = self.get_data_from_state(curr_steering)
        # print(curr_yaw, target_steering)
        # print(target_velocity, curr_vel)

        newAckermannCmd = AckermannDrive()
        newAckermannCmd.speed = target_velocity
        newAckermannCmd.steering_angle = target_steering

        self.controlPub.publish(newAckermannCmd)

    def stop(self):
        newAckermannCmd = AckermannDrive()
        self.controlPub.publish(newAckermannCmd)

    def euler_to_quaternion(self, r):
        (yaw, pitch, roll) = (r[0], r[1], r[2])
        qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(
            roll / 2
        ) * np.sin(pitch / 2) * np.sin(yaw / 2)
        qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(
            roll / 2
        ) * np.cos(pitch / 2) * np.sin(yaw / 2)
        qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(
            roll / 2
        ) * np.sin(pitch / 2) * np.cos(yaw / 2)
        qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(
            roll / 2
        ) * np.sin(pitch / 2) * np.sin(yaw / 2)
        return [qx, qy, qz, qw]

    def quaternion_to_euler(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = np.arcsin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(t3, t4)
        return [roll, pitch, yaw]

    def __waypointHandler(self, data):
        self.waypointList.append(data)

    def __decisionHandler(self, data):
        # print(data.data)
        if data.data in COMMANDs:
            self.state = data.data
        elif data.data in ["ACCEL", "BRAKE"]:
            # print("Hi")
            self.vel_state = data.data
        else:
            print(f"Invalid command: {data.data}")

    def __initHandler(self, data):
        # print(data.data)
        if data.data in ["ACCEL"]:
            self.vel_state = data.data
        else:
            print(f"Invalid command: {data.data}")

