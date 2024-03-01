import time
import math
import numpy as np
import cv2
import rospy
import sys
from line_fit import line_fit, tune_fit, bird_fit, final_viz
import matplotlib.pyplot as plt
from Line import Line
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32
from skimage import morphology
np.set_printoptions(threshold=sys.maxsize)

class lanenet_detector:
    def __init__(self):
        self.bridge = CvBridge()
        # NOTE
        # Uncomment this line for lane detection of GEM car in Gazebo
        self.param = 0
        if self.param == 0:
            self.sub_image = rospy.Subscriber(
                "/front_single_camera/image_raw",
                Image,
                self.img_callback,
                queue_size=1,
            )
        # Uncomment this line for lane detection of videos in rosbag
        elif self.param == 2:
            self.sub_image = rospy.Subscriber('/zed2/zed_node/rgb/image_rect_color', Image, self.img_callback, queue_size=1)
        else:
            self.sub_image = rospy.Subscriber('camera/image_raw', Image, self.img_callback, queue_size=1)
        self.pub_image = rospy.Publisher("lane_detection/annotate", Image, queue_size=1)
        self.pub_bird = rospy.Publisher("lane_detection/birdseye", Image, queue_size=1)
        self.left_line = Line(n=5)
        self.right_line = Line(n=5)
        self.detected = False
        self.hist = True

    def img_callback(self, data):
        try:
            # Convert a ROS image message into an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        raw_img = cv_image.copy()
        mask_image, bird_image = self.detection(raw_img)

        if mask_image is not None and bird_image is not None:
            # Convert an OpenCV image into a ROS image message
            out_img_msg = self.bridge.cv2_to_imgmsg(mask_image, "bgr8")
            out_bird_msg = self.bridge.cv2_to_imgmsg(bird_image, "bgr8")

            # Publish image message in ROS
            self.pub_image.publish(out_img_msg)
            self.pub_bird.publish(out_bird_msg)

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
        if self.param <= 1:
            img_sobelx = cv2.Sobel(img_blur, cv2.CV_64F, 1, 0, ksize=3)
            img_sobely = cv2.Sobel(img_blur, cv2.CV_64F, 0, 1, ksize=3)
        else: 
            # Increase kernel size in order to increase visibility
            img_sobelx = cv2.Sobel(img_blur, cv2.CV_64F, 1, 0, ksize=5)
            img_sobely = cv2.Sobel(img_blur, cv2.CV_64F, 0, 1, ksize=5)
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
        if self.param == 0:
            mask_s = cv2.inRange(img_hsl, (0, thresh[0], 0), (255, thresh[1], 255))
            mask_h = cv2.bitwise_not(cv2.inRange(img_hsl, (75, 0, 0), (85, 255, 255)))
        elif self.param == 3:
            mask_s = cv2.inRange(img_hsl, (0, thresh[0], 0), (255, thresh[1], 255))
            mask_h = cv2.inRange(img_hsl, (0, 0, 10), (255, 255, 100))
        else:
            mask_s = cv2.inRange(img_hsl, (0, thresh[0], 0), (255, thresh[1], 255))
            mask_h = cv2.bitwise_not(cv2.inRange(img_hsl, (50, 0, 0), (130, 255, 255)))
        
        binary_output = cv2.bitwise_and(mask_s, mask_h)

        # plt.figure()
        # plt.imshow(img_hsl[:,:,1])
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
        if self.param == 0:
            SobelOutput = self.gradient_thresh(img, thresh_min=100, thresh_max=150)
            ColorOutput = self.color_thresh(img, thresh=(70, 255))
        elif self.param == 3:
            SobelOutput = self.gradient_thresh(img, thresh_min=250, thresh_max=350)
            ColorOutput = self.color_thresh(img, thresh=(120, 150))
        elif self.param == 2:
            SobelOutput = self.gradient_thresh(img, thresh_min=250, thresh_max=350)
            ColorOutput = self.color_thresh(img, thresh=(200, 255))
        else:
            SobelOutput = self.gradient_thresh(img, thresh_min=140, thresh_max=450)
            ColorOutput = self.color_thresh(img, thresh=(200, 255))

        ####

        binaryImage = np.zeros_like(SobelOutput)
        binaryImage[(ColorOutput >= 1) | (SobelOutput >= 1)] = 1
        # Remove noise from binary image
        if self.param == 2:
            binaryImage = morphology.remove_small_objects(
                binaryImage.astype("bool"), min_size=50, connectivity=2
            )
        else:
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
        if self.param == 0:
            src = np.float32(
                [
                    [220, 280], [400, 280],
                    [0, 400],[640, 400],
                ]
            )
            dst = np.float32([[0, 0], [640, 0], 
                            [0, 480], [640, 480]])
        elif self.param == 1:
            src = np.float32(
                [
                    [500, 225], [750, 225], 
                    [250, 375],[850, 375],
                ]
            )
            dst = np.float32([[0, 0], [640, 0], 
                            [0, 375], [640, 375]])
        elif self.param == 2:
            src = np.float32(
                [
                    [540, 380], [720, 380], 
                    [200, 700],[1050, 700],
                ]
            )
            dst = np.float32([[0, 0], [640, 0], 
                            [0, 375], [640, 375]])
        elif self.param == 3:
            src = np.float32(
                [
                    [600, 270], [800, 270], 
                    [400, 375],[1000, 375],
                ]
            )
            dst = np.float32([[0, 0], [640, 0], 
                            [0, 375], [640, 375]])
        M = cv2.getPerspectiveTransform(src, dst)
        Minv = cv2.getPerspectiveTransform(dst, src)

        img_int = img.astype(np.uint8)
        if self.param == 0:
            warped_img = cv2.warpPerspective(img_int, M, (640, 480))
        elif self.param == 3:
            warped_img = cv2.warpPerspective(img_int, M, (640, 375))
        else:
            warped_img = cv2.warpPerspective(img_int, M, (640, 375))
        
        # cv2.imshow('image', img_int*255)
        # cv2.imshow('warp_image', warped_img*255)
        # cv2.waitKey(0)
        # plt.figure()
        # plt.imshow(img_int)
        # plt.colorbar()
        # plt.savefig("h.png")
        ####

        return warped_img, M, Minv

    def detection(self, img):
        binary_img = self.combinedBinaryImage(img)
        img_birdeye, M, Minv = self.perspective_transform(binary_img)

        if not self.hist:
            # Fit lane without previous result
            ret = line_fit(img_birdeye)
            left_fit = ret["left_fit"]
            right_fit = ret["right_fit"]
            nonzerox = ret["nonzerox"]
            nonzeroy = ret["nonzeroy"]
            left_lane_inds = ret["left_lane_inds"]
            right_lane_inds = ret["right_lane_inds"]

        else:
            # Fit lane with previous result
            if not self.detected:
                ret = line_fit(img_birdeye)

                if ret is not None:
                    left_fit = ret["left_fit"]
                    right_fit = ret["right_fit"]
                    nonzerox = ret["nonzerox"]
                    nonzeroy = ret["nonzeroy"]
                    left_lane_inds = ret["left_lane_inds"]
                    right_lane_inds = ret["right_lane_inds"]

                    left_fit = self.left_line.add_fit(left_fit)
                    right_fit = self.right_line.add_fit(right_fit)

                    self.detected = True

            else:
                left_fit = self.left_line.get_fit()
                right_fit = self.right_line.get_fit()
                ret = tune_fit(img_birdeye, left_fit, right_fit)

                if ret is not None:
                    left_fit = ret["left_fit"]
                    right_fit = ret["right_fit"]
                    nonzerox = ret["nonzerox"]
                    nonzeroy = ret["nonzeroy"]
                    left_lane_inds = ret["left_lane_inds"]
                    right_lane_inds = ret["right_lane_inds"]
                    print(left_lane_inds)

                    left_fit = self.left_line.add_fit(left_fit)
                    right_fit = self.right_line.add_fit(right_fit)

                else:
                    self.detected = False

            # Annotate original image
            bird_fit_img = None
            combine_fit_img = None
            if ret is not None:
                bird_fit_img = bird_fit(img_birdeye, ret, save_file=None)
                combine_fit_img = final_viz(img, left_fit, right_fit, Minv)
            else:
                print("Unable to detect lanes")

            return combine_fit_img, bird_fit_img


if __name__ == "__main__":
    # init args
    rospy.init_node("lanenet_node", anonymous=True)
    lanenet_detector()
    while not rospy.core.is_shutdown():
        rospy.rostime.wallsleep(0.5)
