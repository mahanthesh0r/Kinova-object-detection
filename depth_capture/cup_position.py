#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import message_filters
import tf
import tf.transformations as tf_trans
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient

from kortex_api.autogen.messages import Base_pb2
from kortex_api.Exceptions.KServerException import KServerException
from math import sqrt, inf, degrees, radians
from std_msgs.msg import String

# Constants for image processing
NEW_WIDTH = 480
NEW_HEIGHT = 270

# Example input values (predefined pixel coordinates)
#u = 200  # image coordinate x
#v = 106   # image coordinate y

# Intrinsic parameters matrix
K = np.array([
    [335.055969, 0, 240.156830],
    [0, 335.055969, 137.595566],
    [0, 0, 1]
])

# Extrinsic parameters: Rotation matrix R and translation vector t
R = np.array([
    [1.000100,  0.000000, 0.000000],
    [0.000000,  1.000200, 0.000000],
    [0.000000, 0.000000,  1.000000]
])
t = np.array([-0.027060, -0.009970, -0.004706])

transformation_matrix = np.array([
   [1.000100,  0.000000, 0.000000, -0.027060],
    [0.000000,  1.000200, 0.000000, -0.009970],
    [0.000000, 0.000000,  1.000000, -0.004706]
])

def transformation_matrix_to_euler(transformation_matrix):
    rotation_matrix = transformation_matrix[:3, :3]
    euler_angles = tf_trans.euler_from_matrix(rotation_matrix, axes='sxyz')
    return euler_angles

def calculate_world_coordinates(u, v, Z_c, K, R, t):
    # Convert the image coordinates to normalized camera coordinates
    pixel_coords = np.array([u, v, 1])
    camera_coords = np.linalg.inv(K).dot(pixel_coords) * Z_c

    # Convert from camera coordinates to world coordinates
    world_coords = R.dot(camera_coords) + t

    offset = np.array([0, 56.39, -3.05]) / 1000  # offset from the camera to the end effector
    world_coords += offset

    return world_coords

def reproject_world_to_image(world_coords, K, R, t):
    camera_coords = R.dot(world_coords) + t
    image_coords_homogeneous = K.dot(camera_coords)
    u = image_coords_homogeneous[0] / image_coords_homogeneous[2]
    v = image_coords_homogeneous[1] / image_coords_homogeneous[2]

    return np.array([u, v])

class ImageProcessor:
    def __init__(self):
        self.bridge = CvBridge()
        self.listener = tf.TransformListener()
        self.marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
        # Subscribing to the RGB and depth image topics
        rgb_sub = message_filters.Subscriber("/camera/color/image_raw", Image)
        depth_sub = message_filters.Subscriber("/camera/depth/image_raw", Image)
        
        # Synchronize the image topics
        ts = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub], 10, 0.1)
        ts.registerCallback(self.image_callback)

    def image_callback(self, rgb_data, depth_data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(rgb_data, "bgr8")
            cv_depth_image = self.bridge.imgmsg_to_cv2(depth_data, "32FC1")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
            return

        # Convert the image to the HSV color space
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define the range of red color in HSV
        # Red can be in two ranges, so we define two sets of lower and upper bounds
        lower_red1 = np.array([0, 150, 0])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 150, 0])
        upper_red2 = np.array([180, 255, 255])

        # Threshold the HSV image to get only red colors in both ranges
        mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)

        # Combine the masks for the two red ranges
        mask = cv2.bitwise_or(mask1, mask2)
        
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
                u = int(center_x * NEW_WIDTH / width)
                v = int(center_y * NEW_HEIGHT / height)

        # Resize the image
        resized_image = cv2.resize(cv_image, (NEW_WIDTH, NEW_HEIGHT))

        # Get the depth value for the specified pixel coordinates
        Z_c = cv_depth_image[v, u]
        Z_c = abs(Z_c + 10) # offset from the camera to the end effector  

        if np.isnan(Z_c) or np.isinf(Z_c):
            rospy.logwarn("Invalid depth value at pixel coordinates ({}, {})".format(u, v))
            return

        # Draw a green circle at the predefined coordinates
        cv2.circle(resized_image, (u, v), 5, (0, 255, 0), -1)

        # Calculate world coordinates for the predefined image coordinates (u, v)
        world_coords = calculate_world_coordinates(u, v, Z_c, K, R, t)
        print("Calculated World Coordinates:", world_coords)

        # Convert world_coords from centimeters to meters
        world_coords = np.array(world_coords) / 100

        print("Calculated World Coordinates in Meters:", world_coords)

        # Reproject the world coordinates back to image coordinates
        reprojected_coords = reproject_world_to_image(world_coords, K, R, t)
        print("Reprojected Image Coordinates:", reprojected_coords)

        # Calculate reprojection error
        reprojection_error = np.linalg.norm(np.array([u, v]) - reprojected_coords)
        print("Reprojection Error:", reprojection_error)

        # Draw a red circle at the reprojected image coordinates
        u_reproj, v_reproj = map(int, reprojected_coords)
        cv2.circle(resized_image, (u_reproj, v_reproj), 5, (0, 0, 255), -1)

        # Convert world coordinates to robot frame coordinates
        robot_frame_coords = self.transform_to_robot_frame(world_coords)
        print("Robot Frame Coordinates:", robot_frame_coords)

        # Publish marker at the transformed point
        if robot_frame_coords is not None:
            self.publish_marker(robot_frame_coords)

        # Optionally, display the image using OpenCV (useful for debugging)
        #cv2.imshow("Processed Image", resized_image)
        cv2.imwrite("filename.jpg", resized_image)

    
    def quaternion_to_axis_angle(self, quaternion):
        """
        Convert a quaternion to axis-angle representation.
        Quaternion format: (x, y, z, w)
        Returns: (axis, angle) where axis is a 3D unit vector and angle is in radians.
        """
        w, x, y, z = quaternion
        angle = 2 * np.arccos(w)
        s = np.sqrt(1 - w**2)
        if s < 0.001:
            # To avoid division by zero, if s is close to zero then direction of the axis is not important
            x = y = z = 1
        else:
            x = x / s
            y = y / s
            z = z / s
        axis = np.array([x, y, z])
        return axis, angle
    
    def normalize_quaternion(self, quaternion):
        norm = np.linalg.norm(quaternion)
        if norm == 0:
            raise ValueError("Cannot normalize a quaternion with zero norm")
        return quaternion / norm

    def transform_to_robot_frame(self, world_coords):
       
        pub = rospy.Publisher('object_location', String, queue_size=10)
        

        # Wait for the transformation to be available
        self.listener.waitForTransform("/base_link", "/camera_link", rospy.Time(0), rospy.Duration(4.0))

        # Create a TransformStamped object with the world coordinates
        world_point = tf.transformations.translation_matrix(world_coords)
        world_point[3][3] = 1.0  # homogeneous coordinate
        
        print("World Point: ", world_point)

        # Get the transformation from the world frame to the robot frame
        try:
            (trans, rot) = self.listener.lookupTransform('/base_link', '/camera_link', rospy.Time(0))

           
            print("Normalized Quaternion: ", rot)

            transform = tf.transformations.concatenate_matrices(
                tf.transformations.translation_matrix(trans),
                tf.transformations.quaternion_matrix(rot)
            )
            print("Transform: ", transform)
            # Transform the world coordinates to the robot frame
            robot_point = np.dot(transform, world_point)
             
            print("Robot Point: ", robot_point)

            print("Quaternion: ", rot)
            axis, angle = self.quaternion_to_axis_angle(rot)
            print("Axis: ", axis)
            print("Angle (radians): ", angle)
            euler_angles = tf.transformations.euler_from_quaternion(rot)

            print("Euler Angles: ", degrees(euler_angles[0]), degrees(euler_angles[1]), degrees(euler_angles[2]))

            #Publish the robot point and euler angles
            object_location = "x: " + str(robot_point[0][3]) + " y: " + str(robot_point[1][3]) + " z: " + str(robot_point[2][3]) + " roll: " + str(degrees(euler_angles[0])) + " pitch: " + str(degrees(euler_angles[1])) + " yaw: " + str(degrees(euler_angles[2]))
            rospy.loginfo("Publishing object location: %s", object_location)
            pub.publish(object_location)

            rospy.sleep(1)
            
            return robot_point[:3, 3]  # extract the (x, y, z) coordinates
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("Transform lookup failed")
            return None
        
    
    def publish_marker(self, robot_frame_coords):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "robot_frame"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = robot_frame_coords[0] 
        marker.pose.position.y = robot_frame_coords[1] 
        marker.pose.position.z = robot_frame_coords[2]
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05  # Size of the sphere
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0  # Alpha channel
        marker.color.r = 1.0  # Red
        marker.color.g = 0.0  # Green
        marker.color.b = 0.0  # Blue

        self.marker_pub.publish(marker)     

def main():
    rospy.init_node('image_processor', anonymous=True)
    ip = ImageProcessor()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
