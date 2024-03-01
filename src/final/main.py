import rospy
import math
import argparse
from pynput import keyboard

from controller import bicycleModel
import time

# from studentVision import lanenet_detector


def run_model():
    rospy.init_node("model_dynamics")
    model = bicycleModel(velocity=3.0, deceleration=0.0)

    def shutdown():
        """Stop the car when this ROS node shuts down"""
        model.stop()
        rospy.loginfo("Stop the car")

    rospy.on_shutdown(shutdown)

    rate = rospy.Rate(100)  # 100 Hz

    # listener = keyboard.Listener(on_press=on_key_press)
    # listener.start()

    while not rospy.is_shutdown():
        rate.sleep()  # Wait a while before trying to get a new state

        # Get the current position and orientation of the vehicle
        currState = model.getModelState()

        if not currState.success:
            continue
        # print(state)

        model.setModelState(currState, "run")


if __name__ == "__main__":
    try:
        run_model()
    except rospy.exceptions.ROSInterruptException:
        rospy.loginfo("Shutting down")
