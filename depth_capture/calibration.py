#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class ImageProcessor:
    def __init__(self):
        rospy.init_node('image_processor', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback)
        self.image_pub = rospy.Publisher("/camera/image_processed", Image, queue_size=10)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

            # Get the pixel dimensions
        height, width, channels = cv_image.shape
        print("Image dimensions - Width:", width, "Height:", height)

        # Draw a small red dot on specific pixel
        pixel_x = 618  # Example pixel x-coordinate
        pixel_y = 236  # Example pixel y-coordinate
        cv2.circle(cv_image, (pixel_x, pixel_y), 3, (0, 0, 255), -1)

        pixel_x2 = 19 * 32
        pixel_y2 = 7 * 32
        cv2.circle(cv_image, (pixel_x2, pixel_y2), 5, (255, 0, 0), -1)

        # Save the processed image
        # filename = "processed_image.jpg"
        # cv2.imwrite(filename, cv_image)
        # print("Processed image saved as:", filename)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)


def main():
    image_processor = ImageProcessor()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()
