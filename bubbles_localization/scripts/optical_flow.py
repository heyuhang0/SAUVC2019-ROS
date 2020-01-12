#!/usr/bin/env python2
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class OpticalFlow:
    def __init__(self):
        rospy.init_node('gate_tracker')

        # Create the cv_bridge object
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('image', Image, self.on_new_frame)

    def on_new_frame(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        frame = np.array(frame, dtype=np.uint8)
        self.optical_flow(cv2.resize(frame, (100, 100)))

    def optical_flow(self, frame):
        if not hasattr(self, 'first_frame'):
            # ret = a boolean return value from getting the frame, first_frame = the first frame in the entire video sequence
            self.first_frame = frame
            # Converts frame to grayscale because we only need the luminance channel for detecting edges - less computationally expensive
            self.prev_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            # Creates an image filled with zero intensities with the same dimensions as the frame
            self.mask = np.zeros_like(frame)
            # Sets image saturation to maximum
            self.mask[..., 1] = 255
            self.x = 0
            self.y = 0
            return
        # Opens a new window and displays the input frame
        cv2.imshow("input", frame)
        # Converts each frame to grayscale - we previously only converted the first frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # Calculates dense optical flow by Farneback method
        # https://docs.opencv2.org/3.0-beta/modules/video/doc/motion_analysis_and_object_tracking.html#calcopticalflowfarneback
        flow = cv2.calcOpticalFlowFarneback(self.prev_gray, gray, None, 0.5, 3, 15, 3, 5, 1.2, 0)
        # Computes the magnitude and angle of the 2D vectors
        magnitude, angle = cv2.cartToPolar(flow[..., 0], flow[..., 1])
        # Sets image hue according to the optical flow direction
        self.mask[..., 0] = angle * 180 / np.pi / 2
        # Sets image value according to the optical flow magnitude (normalized)
        self.mask[..., 2] = cv2.normalize(magnitude, None, 0, 255, cv2.NORM_MINMAX)
        # Converts HSV to RGB (BGR) color representation
        rgb = cv2.cvtColor(self.mask, cv2.COLOR_HSV2BGR)
        # Opens a new window and displays the output frame
        cv2.imshow("dense optical flow", rgb)
        # Updates previous frame
        self.prev_gray = gray
        # Frames are read by intervals of 1 millisecond
        cv2.waitKey(1)

        avg_vel = np.average(np.average(flow, 0), 0)
        vy, vx = avg_vel
        self.x += vx / 30.0
        self.y += vy / 30.0
        print("vx:{:4.1f}\tx:{:4.1f}\tvy:{:4.1f}\ty:{:4.1f}".format(vx, self.x, vy, self.y))


if __name__ == "__main__":
    OpticalFlow()
    rospy.spin()
