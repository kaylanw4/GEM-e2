import time
import math
import numpy as np
import cv2
import rospy
import sys
from line_fit import (
    line_fit_left,
    line_fit_right,
    line_fit,
    tune_fit_left,
    tune_fit_right,
    bird_fit,
    final_viz,
)
import matplotlib.pyplot as plt
from Line import Line
from sensor_msgs.msg import Image
from std_msgs.msg import Header, String
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32
from skimage import morphology
from msgParser import dumpLaneMsg

np.set_printoptions(threshold=sys.maxsize)


class lanenet_detector:
    def __init__(self):
        self.bridge = CvBridge()
        # NOTE
        # Uncomment this line for lane detection of GEM car in Gazebo
        self.sub_image = rospy.Subscriber(
            "/zed2/zed_node/rgb/image_rect_color",
            Image,
            self.img_callback,
            queue_size=1,
        )
        # Uncomment this line for lane detection of videos in rosbag
        self.pub_image = rospy.Publisher("lane_detection/annotate", Image, queue_size=1)
        self.pub_bird = rospy.Publisher("lane_detection/birdseye", Image, queue_size=1)
        self.pub_data = rospy.Publisher("lane_detection/data", String, queue_size=1)
        self.left_line = Line(n=5)
        self.right_line = Line(n=5)
        self.detected_left = False
        self.detected_right = False
        self.hist = True
        self.state = "BOTH"  # "LEFT", "RIGHT", "OUT"

    def img_callback(self, data):
        try:
            # Convert a ROS image message into an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        raw_img = cv_image.copy()
        mask_image, bird_image, left_fit, right_fit, img_shape = self.detection(raw_img)

        if mask_image is not None and bird_image is not None:
            # Convert an OpenCV image into a ROS image message
            out_img_msg = self.bridge.cv2_to_imgmsg(mask_image, "bgr8")
            out_bird_msg = self.bridge.cv2_to_imgmsg(bird_image, "bgr8")

            # Publish image message in ROS
            self.pub_image.publish(out_img_msg)
            self.pub_bird.publish(out_bird_msg)
            # rospy.loginfo(msg_data)
            msg_data = dumpLaneMsg(self.state, left_fit, right_fit, img_shape)
            self.pub_data.publish(msg_data)

    def gradient_thresh(self, img, thresh_min=25, thresh_max=100):
        """
        Apply sobel edge detection on input image in x, y direction
        """
        # print(img)
        ## TODO
        # 1. Convert the image to gray scale
        img_edit = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # 2. Gaussian blur the image
        img_blur = cv2.GaussianBlur(img_edit, (5, 5), 0)

        # 3. Use cv2.Sobel() to find derievatives for both X and Y Axis
        img_sobelx = cv2.Sobel(img_blur, cv2.CV_64F, 1, 0, ksize=3)
        img_sobely = cv2.Sobel(img_blur, cv2.CV_64F, 0, 1, ksize=3)
        img_absX = cv2.convertScaleAbs(img_sobelx)
        img_absY = cv2.convertScaleAbs(img_sobely)

        # 4. Use cv2.addWeighted() to combine the results
        alpha = 0.5
        beta = 1 - alpha
        img_weight = cv2.addWeighted(img_absX, alpha, img_absY, beta, 0.0)

        # 5. Convert each pixel to uint8, then apply threshold to get binary image
        img_uint = img_weight.astype(np.uint8)
        ret, binary_output = cv2.threshold(
            img_uint, thresh_min, thresh_max, cv2.THRESH_BINARY
        )

        # cv2.imshow('image', binary_output)
        # cv2.waitKey(0)

        # plt.figure()
        # plt.imshow(img_weight)
        # plt.colorbar()
        # plt.savefig("h.png")
        ####

        return binary_output

    def color_thresh(self, img, thresh=(70, 255)):
        """
        Convert RGB to HSL and threshold to binary image using S channel
        """
        # 1. Convert the image from RGB to HSL
        # 2. Apply threshold on S channel to get binary image
        # Hint: threshold on H to remove green grass
        ## TODO
        img_hsl = cv2.cvtColor(img, cv2.COLOR_RGB2HLS)
        mask_s = cv2.inRange(img_hsl, (0, thresh[0], 0), (255, thresh[1], 255))
        mask_h = cv2.bitwise_not(cv2.inRange(img_hsl, (75, 0, 0), (120, 255, 255)))

        binary_output = cv2.bitwise_and(mask_s, mask_h)

        # plt.figure()
        # plt.imshow(img_hsl[:,:,0])
        # plt.colorbar()
        # plt.savefig("h.png")
        # cv2.imshow('ori_image', img)
        # cv2.imshow('mask_s', mask_s)
        # cv2.imshow('mask_h', mask_h)
        # cv2.imshow('image', binary_output)
        # cv2.waitKey(0)
        ####

        return binary_output

    def combinedBinaryImage(self, img):
        """
        Get combined binary image from color filter and sobel filter
        """
        # 1. Apply sobel filter and color filter on input image
        # 2. Combine the outputs
        ## Here you can use as many methods as you want.

        ## TODO
        SobelOutput = self.gradient_thresh(img, thresh_min=100, thresh_max=150)
        ColorOutput = self.color_thresh(img, thresh=(70, 125))

        ####

        binaryImage = np.zeros_like(SobelOutput)
        binaryImage[(ColorOutput >= 1) | (SobelOutput >= 1)] = 1
        # Remove noise from binary image
        binaryImage = morphology.remove_small_objects(
            binaryImage.astype("bool"), min_size=10, connectivity=2
        )

        # cv2.imshow('sobel', SobelOutput)
        # cv2.imshow('color', ColorOutput)
        # cv2.imshow('image', binaryImage.astype(np.uint8)*255)
        # cv2.waitKey(0)

        return binaryImage

    def perspective_transform(self, img, verbose=False):
        """
        Get bird's eye view from input image
        """
        # 1. Visually determine 4 source points and 4 destination points
        # 2. Get M, the transform matrix, and Minv, the inverse using cv2.getPerspectiveTransform()
        # 3. Generate warped image in bird view using cv2.warpPerspective()

        ## TODO
        # print(img.shape)
        src = np.float32(
            [
                [500, 400],
                [720, 400],
                [200, 700],
                [1080, 700],
            ]
        )
        dst = np.float32([[0, 0], [640, 0], [0, 480], [640, 480]])

        M = cv2.getPerspectiveTransform(src, dst)
        Minv = cv2.getPerspectiveTransform(dst, src)

        img_int = img.astype(np.uint8)
        warped_img = cv2.warpPerspective(img_int, M, (640, 480))

        # cv2.imshow('image', img_int*255)
        # cv2.imshow('warp_image', warped_img*255)
        # cv2.waitKey(0)
        # plt.figure()
        # plt.imshow(img_int)
        # plt.colorbar()
        # plt.savefig("h.png")
        ####

        return warped_img, M, Minv

    def guess_direction(self, ret_l, ret_r, binary_warped):
        nonzerox = ret_l["nonzerox"]
        nonzeroy = ret_l["nonzeroy"]
        left_fit = ret_l["left_fit"]
        right_fit = ret_r["right_fit"]
        left_lane_inds = ret_l["left_lane_inds"]
        right_lane_inds = ret_r["right_lane_inds"]
        ploty_high = np.linspace(
            0, binary_warped.shape[0] // 2 - 1, binary_warped.shape[0] // 2
        )
        ploty_low = np.linspace(
            binary_warped.shape[0] // 2,
            binary_warped.shape[0] - 1,
            binary_warped.shape[0] // 2,
        )
        left_fitx_low = (
            left_fit[0] * ploty_low**2 + left_fit[1] * ploty_low + left_fit[2]
        )
        left_fitx_high = (
            left_fit[0] * ploty_high**2 + left_fit[1] * ploty_high + left_fit[2]
        )
        right_fitx_low = (
            right_fit[0] * ploty_low**2 + right_fit[1] * ploty_low + right_fit[2]
        )
        right_fitx_high = (
            right_fit[0] * ploty_high**2 + right_fit[1] * ploty_high + right_fit[2]
        )

        # print(left_fitx_high.mean(), left_fitx_low.mean())
        if left_fitx_high.mean() - left_fitx_low.mean() > 30:
            dir_l = "R"
        elif left_fitx_high.mean() - left_fitx_low.mean() < -30:
            dir_l = "L"
        else:
            dir_l = "N"
        if right_fitx_high.mean() - right_fitx_low.mean() > 30:
            dir_r = "R"
        elif right_fitx_high.mean() - right_fitx_low.mean() < -30:
            dir_r = "L"
        else:
            dir_r = "N"

        if dir_l == "R" and dir_r != "L":
            return "Right"
        if dir_l == "L" and dir_r != "R":
            return "Left"
        return None

    def detection(self, img):
        binary_img = self.combinedBinaryImage(img)
        img_birdeye, M, Minv = self.perspective_transform(binary_img)
        # print(self.state)

        if not self.hist:
            # Fit lane without previous result
            ret = line_fit(img_birdeye)
            left_fit = ret["left_fit"]
            right_fit = ret["right_fit"]

        else:
            # Fit lane with previous result
            # print(self.detected_left, self.detected_right)
            if not self.detected_left:
                ret_l = line_fit_left(img_birdeye)

                if ret_l is not None:
                    left_fit = ret_l["left_fit"]
                    left_fit = self.left_line.add_fit(left_fit)
                    left_lane_idx = ret_l["left_lane_inds"]
                    detected_left = True
                else:
                    detected_left = False

            else:
                left_fit = self.left_line.get_fit()
                ret_l = tune_fit_left(img_birdeye, left_fit)

                if ret_l is not None:
                    left_fit = ret_l["left_fit"]
                    left_fit = self.left_line.add_fit(left_fit)
                    left_lane_idx = ret_l["left_lane_inds"]
                    detected_left = True

                else:
                    detected_left = False

            if not self.detected_right:
                ret_r = line_fit_right(img_birdeye)

                if ret_r is not None:
                    right_fit = ret_r["right_fit"]
                    right_fit = self.right_line.add_fit(right_fit)
                    right_lane_idx = ret_r["right_lane_inds"]
                    detected_right = True
                else:
                    detected_right = False

            else:
                right_fit = self.right_line.get_fit()
                ret_r = tune_fit_right(img_birdeye, right_fit)

                if ret_r is not None:
                    right_fit = ret_r["right_fit"]
                    right_fit = self.right_line.add_fit(right_fit)
                    right_lane_idx = ret_r["right_lane_inds"]
                    detected_right = True

                else:
                    detected_right = False

            if detected_left and detected_right:
                # print(left_fit, right_fit)
                # if np.sum(left_lane_idx & right_lane_idx) > 30:
                # print(left_lane_idx)
                # print(len(np.intersect1d(left_lane_idx, right_lane_idx))

                if np.sum(left_lane_idx & right_lane_idx) > 30:
                    # if len(np.intersect1d(left_lane_idx, right_lane_idx)) > 30:
                    # Two lane overlap
                    # print("hit")
                    if self.state == "LEFT":
                        ret_r = None
                        self.detected_left = True
                        self.detected_right = False
                    elif self.state == "RIGHT":
                        ret_l = None
                        self.detected_left = False
                        self.detected_right = True
                    else:
                        direction = self.guess_direction(ret_l, ret_r, img_birdeye)
                        # print(direction)
                        if direction:
                            if direction == "Right":
                                ret_r = None
                                self.detected_left = True
                                self.detected_right = False
                                self.state = "LEFT"
                            else:
                                ret_l = None
                                self.detected_left = False
                                self.detected_right = True
                                self.state = "RIGHT"
                        else:
                            ret_l = None
                            ret_r = None
                            self.detected_left = False
                            self.detected_right = False
                            # pass
                else:
                    self.state = "BOTH"
                    self.detected_left = True
                    self.detected_right = True
            elif detected_left:
                self.state = "LEFT"
                ret_r = None
                self.detected_left = True
                self.detected_right = False
            elif detected_right:
                self.state = "RIGHT"
                ret_l = None
                self.detected_left = False
                self.detected_right = True
            else:
                self.state = "OUT"
                ret_l = None
                ret_r = None
                self.detected_left = False
                self.detected_right = False

            # # If two line are identicle, return all None to trigger lint_fit()
            # if np.sum(left_lane_inds & right_lane_inds)>30:
            #     return None

            # Annotate original image
            # print(self.state, ret_l, ret_r)
            bird_fit_img = None
            combine_fit_img = None
            if ret_r is not None and ret_l is not None:
                bird_fit_img = bird_fit(img_birdeye, ret_l, ret_r, save_file=None)
                combine_fit_img = final_viz(img, left_fit, right_fit, Minv)
            elif ret_r is not None:
                bird_fit_img = bird_fit(img_birdeye, None, ret_r, save_file=None)
                combine_fit_img = final_viz(img, None, right_fit, Minv)
                left_fit = None
            elif ret_l is not None:
                bird_fit_img = bird_fit(img_birdeye, ret_l, None, save_file=None)
                combine_fit_img = final_viz(img, left_fit, None, Minv)
                right_fit = None
            else:
                print("[StudentVision] Unable to detect lanes")
                left_fit = None
                right_fit = None
            # print(combine_fit_img)

            return (
                combine_fit_img,
                bird_fit_img,
                left_fit,
                right_fit,
                img_birdeye.shape,
            )


if __name__ == "__main__":
    # init args
    rospy.init_node("lanenet_node", anonymous=True)
    lanenet_detector()
    while not rospy.core.is_shutdown():
        rospy.rostime.wallsleep(0.5)
