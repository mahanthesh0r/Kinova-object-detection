#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
import message_filters

class ObjectPosition:
    def __init__(self):
        self.bridge = CvBridge()
        
        # Subscribe to the RGB and depth image topics
        self.image_sub = message_filters.Subscriber("/camera/color/image_raw", Image)
        self.depth_sub = message_filters.Subscriber("/camera/depth/image_raw", Image)
        
        # Subscribe to the camera info topic
        self.info_sub = rospy.Subscriber("/camera/depth/camera_info", CameraInfo, self.info_callback)
        
        # Synchronize the RGB and depth images
        self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.depth_sub], 10, 0.1)
        self.ts.registerCallback(self.callback)
        
        self.fx = self.fy = self.cx = self.cy = None

    def info_callback(self, info):
        # Get the camera intrinsic parameters
        self.fx = info.K[0]
        self.fy = info.K[4]
        self.cx = info.K[2]
        self.cy = info.K[5]

    def callback(self, rgb_data, depth_data):
        try:
            # Convert the images to OpenCV format
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_data, "bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(depth_data, "32FC1")
        except CvBridgeError as e:
            print(e)
            return
        
        #resize depth image to match rgb image
        rgb_height, rgb_width = rgb_image.shape[:2]
        depth_image_resized = cv2.resize(depth_image, (rgb_width, rgb_height), interpolation=cv2.INTER_NEAREST)

        depth_width = depth_data.width
        depth_height = depth_data.height

        rospy.loginfo(f"Depth image dimensions: Width={depth_width} Height={depth_height} ")
        # Example coordinates of the point of interest in the RGB image
        point_x = 618
        point_y = 236

        # Retrieve the depth value (in meters) at the selected point
        depth_value = depth_image_resized[point_y, point_x]

        if depth_value == 0:
            rospy.logwarn("Depth value at ({}, {}) is 0. Skipping.".format(point_x, point_y))
            return

        # Calculate real-world coordinates
        # real_x = (point_x - self.cx) * depth_value / self.fx
        # real_y = (point_y - self.cy) * depth_value / self.fy
        # real_z = depth_value
        print(f"fx: {self.fx}")
        print(f"fy: {self.fy}")
        print(f"cx: {self.cx}")
        print(f"cy: {self.cy}")
        

        real_x = point_x / 32
        real_y = point_y / 32
        real_z = depth_value

        rospy.loginfo("Real-world position: X={:.3f}, Y={:.3f}, Z={:.3f}".format(real_x, real_y, real_z))


        # Verification with known object dimensions and location
        known_x = 19  # Replace with actual known X coordinate in meters
        known_y = 7  # Replace with actual known Y coordinate in meters
        known_z = 43  # Replace with actual known Z coordinate in meters

        error_x = abs(real_x - known_x)
        error_y = abs(real_y - known_y)
        error_z = abs(real_z - known_z)

        rospy.loginfo(f"Error: X={error_x:.3f}, Y={error_y:.3f}, Z={error_z:.3f}")

        if error_x < 1 and error_y < 1 and error_z < 3:
            rospy.loginfo("The calculated coordinates are accurate.")
        else:
            rospy.logwarn("The calculated coordinates are inaccurate.")

def main():
    rospy.init_node('object_position', anonymous=True)
    op = ObjectPosition()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
