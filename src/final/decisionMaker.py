import time
import math
import numpy as np
import cv2
import rospy
import sys
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image
from std_msgs.msg import Header, String
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32
from skimage import morphology
from pynput import keyboard
from msgParser import parseLaneMsg

np.set_printoptions(threshold=sys.maxsize)


class decision_maker:
    def __init__(self):
        self.pub_data = rospy.Publisher("decision_maker/data", String, queue_size=1)
        self.decisionMakerSub = rospy.Subscriber(
            "/lane_detection/data", String, self.__laneHandler, queue_size=1
        )
        self.pub_init = rospy.Publisher("decision_maker/init_data", String, queue_size=1)
        # self.rightx_base = 500
        # self.leftx_base = 100
        # self.range = 50
        self.rightx_turn_base = 440
        self.leftx_turn_base = 200
        self.rightx_straight_base = 560
        self.leftx_straight_base = 80
        self.range = 50
        self.start = False

    def decision(self, lane):
        state = lane["lane"]
        left_fit = lane["left_fit"]
        right_fit = lane["right_fit"]
        y = lane["img_height"][0]
        x = lane["img_height"][1]
        if state == "RIGHT":
            rightx = right_fit[0] * y**2 + right_fit[1] * y + right_fit[2]
            if rightx > x and right_fit[2] < 0:
                return state, "HARDLEFT"
            elif rightx > self.rightx_turn_base + self.range:
                return state, "STRAIGHT"
            elif rightx < self.rightx_turn_base - self.range:
                return state, "HARDLEFT"
            else:
                return state, "LEFT"
        elif state == "LEFT":
            leftx = left_fit[0] * y**2 + left_fit[1] * y + left_fit[2]
            if leftx < 0 and left_fit[2] > x:
                return state, "HARDRIGHT"
            elif leftx > self.leftx_turn_base + self.range:
                return state, "HARDRIGHT"
            elif leftx < self.leftx_turn_base - self.range:
                return state, "STRAIGHT"
            else:
                return state, "RIGHT"
        elif state == "BOTH":
            leftx = left_fit[0] * y**2 + left_fit[1] * y + left_fit[2]
            rightx = right_fit[0] * y**2 + right_fit[1] * y + right_fit[2]
            if leftx < 0:
                if rightx > self.rightx_straight_base + self.range:
                    return state, "RIGHT"
                elif rightx < self.rightx_straight_base - self.range:
                    return state, "LEFT"
                else:
                    return state, "STRAIGHT"
            if rightx > x:
                if leftx > self.leftx_straight_base + self.range:
                    return state, "RIGHT"
                elif leftx < self.leftx_straight_base - self.range:
                    return state, "LEFT"
                else:
                    return state, "STRAIGHT"
            if leftx + rightx > x + self.range:
                return state, "RIGHT"
            elif leftx + rightx < x - self.range:
                return state, "LEFT"
            else:
                return state, "STRAIGHT"
        else:
            return state, "BRAKE"

    def __laneHandler(self, data):
        lane = parseLaneMsg(data.data)
        state, decision = self.decision(lane)
        # print(state, decision)
        self.pub_init.publish("ACCEL")
        self.pub_data.publish(decision)


if __name__ == "__main__":
    # init args
    rospy.init_node("lanenet_node", anonymous=True)
    decision_maker()
    while not rospy.core.is_shutdown():
        rospy.rostime.wallsleep(0.5)
