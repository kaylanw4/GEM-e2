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

np.set_printoptions(threshold=sys.maxsize)


class lanenet_detector:
    def __init__(self):
        self.pub_data = rospy.Publisher("decision_maker/data", String, queue_size=1)
        listener = keyboard.Listener(on_press=self.on_key_press)
        listener.start()

    def on_key_press(self, key):
        print(key)
        # keyboard mode
        if key == keyboard.Key.up:
            msg_data = "ACCEL"
        elif key == keyboard.Key.left:
            msg_data = "LEFT"
        elif key == keyboard.Key.right:
            msg_data = "RIGHT"
        elif key == keyboard.Key.down:
            msg_data = "BRAKE"
        else:
            msg_data = "BRAKE"
            try:
                if key.char == "a":
                    msg_data = "HARDLEFT"
                elif key.char == "d":
                    msg_data = "HARDRIGHT"
                elif key.char == "w":
                    msg_data = "STRAIGHT"
            except:
                msg_data = "BRAKE"
        self.pub_data.publish(msg_data)


if __name__ == "__main__":
    # init args
    rospy.init_node("lanenet_node", anonymous=True)
    lanenet_detector()
    while not rospy.core.is_shutdown():
        rospy.rostime.wallsleep(0.5)
