#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class BlueBallTracker:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        self.scaled_width = 480
        self.scaled_height = 270

    def image_callback(self, data):
        try:
            # Convert the image from ROS format to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
            return
        
        # Convert the image to the HSV color space
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # Define the range of blue color in HSV
        lower_blue = np.array([100, 150, 0])
        upper_blue = np.array([140, 255, 255])
        
        # Threshold the HSV image to get only blue colors
        mask = cv2.inRange(hsv_image, lower_blue, upper_blue)
        
        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            # Find the largest contour
            c = max(contours, key=cv2.contourArea)
            
            # Get the center of the contour
            M = cv2.moments(c)
            if M["m00"] != 0:
                center_x = int(M["m10"] / M["m00"])
                center_y = int(M["m01"] / M["m00"])
                
                # Draw a red dot at the center
                cv2.circle(cv_image, (center_x, center_y), 5, (0, 0, 255), -1)
                
                # Scale the coordinates
                height, width, _ = cv_image.shape
                scaled_x = int(center_x * self.scaled_width / width)
                scaled_y = int(center_y * self.scaled_height / height)
                
                # Print the scaled coordinates
                rospy.loginfo("Red dot at: ({}, {})".format(scaled_x, scaled_y))
        
        # Display the image
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

def main():
    rospy.init_node('blue_ball_tracker', anonymous=True)
    bbt = BlueBallTracker()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
