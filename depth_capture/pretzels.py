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
import math

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

    # offset = np.array([0, 56.39, -3.05])  # offset from the camera to the end effector
    # world_coords += offset

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

        self.pretzels_angle = 0



    
    def calculate_pretzel_orientation(self, largest_brown_contour, image):
        """
        Calculate the orientation of the pretzel from its contour.
        Returns: angle of rotation in radians
        """
        # Fit an ellipse to the contour (this method requires at least 5 points)
        if len(largest_brown_contour) >= 5:
            ellipse = cv2.fitEllipse(largest_brown_contour)
            center, axes, angle = ellipse
            cx, cy = int(center[0]), int(center[1])  # Center of the ellipse
            major_axis_length = max(axes) / 2  # Half of the major axis length
            minor_axis_length = min(axes) / 2  # Half of the minor axis length

            # Adjust the angle by adding 90 degrees to align with the robot's end-effector
            # if angle > 80 and angle < 100:
            #     angle -= 90
            # elif angle > 0 and angle < 30:
            #     angle = 90

            # The angle is in degrees, convert it to radians
            angle_rad = math.radians(angle)

            # Draw the major axis (in the direction of the pretzel's orientation)
            major_axis_x = int(cx + major_axis_length * math.cos(angle_rad))
            major_axis_y = int(cy + major_axis_length * math.sin(angle_rad))
            cv2.line(image, (cx, cy), (major_axis_x, major_axis_y), (0, 255, 0), 2)  # Green line for major axis

            # Draw the minor axis (perpendicular to the major axis)
            minor_axis_x = int(cx - minor_axis_length * math.sin(angle_rad))
            minor_axis_y = int(cy + minor_axis_length * math.cos(angle_rad))
            cv2.line(image, (cx, cy), (minor_axis_x, minor_axis_y), (255, 0, 0), 2)  # Blue line for minor axis

            # Draw the center of the ellipse
            cv2.circle(image, (cx, cy), 5, (0, 0, 255), -1)  # Red dot at the center

            return angle_rad, center, angle
        else:
            return None, None, None

        

    def image_callback(self, rgb_data, depth_data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(rgb_data, "bgr8")
            cv_depth_image = self.bridge.imgmsg_to_cv2(depth_data, "32FC1")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
            return

        # Convert the image to the HSV color space
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define the range of white color in HSV
        lower_white = np.array([0, 0, 0])
        upper_white = np.array([180, 50, 255])

        # Create a mask for the white color
        white_mask = cv2.inRange(hsv_image, lower_white, upper_white)

        # Apply Gaussian blur to the mask to reduce noise
        white_mask = cv2.GaussianBlur(white_mask, (5, 5), 0)

        # Apply morphological operations to clean up the mask
        kernel = np.ones((5, 5), np.uint8)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, kernel)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel)

        # Find contours in the mask
        contours, _ = cv2.findContours(white_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Identify the largest contour
            largest_contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour)

            # Draw the bounding box around the white contour (plate)
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (255, 0, 0), 2)  # Blue bounding box

            # Define the range of brown color in HSV
            lower_brown = np.array([10, 100, 20])
            upper_brown = np.array([30, 255, 200])

            # Create a mask for the brown color within the bounding box of the plate
            plate_region = hsv_image[y:y+h, x:x+w]
            brown_mask = cv2.inRange(plate_region, lower_brown, upper_brown)

            # Apply Gaussian blur to the mask to reduce noise
            brown_mask = cv2.GaussianBlur(brown_mask, (5, 5), 0)

            # Apply morphological operations to clean up the mask
            kernel = np.ones((5, 5), np.uint8)
            brown_mask = cv2.morphologyEx(brown_mask, cv2.MORPH_CLOSE, kernel)
            brown_mask = cv2.morphologyEx(brown_mask, cv2.MORPH_OPEN, kernel)

            # Find contours in the brown mask
            brown_contours, _ = cv2.findContours(brown_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            if brown_contours:

                # Identify the largest brown contour
                largest_brown_contour = max(brown_contours, key=cv2.contourArea)

                # Calculate the center and orientation of the largest brown contour
                angle_rad, pretzel_center, pretzel_angle = self.calculate_pretzel_orientation(largest_brown_contour, cv_image)

                if pretzel_center is not None:
                    # Calculate orientation of the pretzel and adjust the robot's end-effector accordingly
                    rospy.loginfo(f"Orientation of the pretzel: {math.degrees(angle_rad)} degrees")
                    self.pretzel_angle = angle_rad

                    # Adjust the robot's orientation based on the pretzel orientation
                    robot_orientation_quaternion = tf.transformations.quaternion_from_euler(0, 0, angle_rad)
                    rospy.loginfo(f"Robot Quaternion for adjustment: {robot_orientation_quaternion}")

                # Calculate the center of the largest contour
                M = cv2.moments(largest_brown_contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"]) + x
                    cY = int(M["m01"] / M["m00"]) + y

                    # Draw a red dot at the center
                    cv2.circle(cv_image, (cX, cY), 5, (0, 0, 255), -1)

                    # Calculate the bounding box for the largest brown contour
                    bx, by, bw, bh = cv2.boundingRect(largest_brown_contour)

                    # Draw the bounding box around the brown pretzel
                    cv2.rectangle(cv_image, (x + bx, y + by), (x + bx + bw, y + by + bh), (0, 255, 0), 2)
                    
                    # Scale the coordinates
                    height, width, _ = cv_image.shape
                    u = int(cX * NEW_WIDTH / width)
                    v = int(cY * NEW_HEIGHT / height)
                else:
                    cX, cY = 0, 0

            # Use the center coordinates (cX, cY) as needed
            rospy.loginfo(f"Center of the pretzel: ({cX}, {cY})")
        else:
            rospy.loginfo("No pretzel found")

        # Resize the image
        resized_image = cv2.resize(cv_image, (NEW_WIDTH, NEW_HEIGHT))

        # Get the depth value for the specified pixel coordinates
        Z_c = cv_depth_image[v, u]
        

        #print("Depth Value (Z_c):", Z_c - 20)
        #Z_c = abs(Z_c - 25) # offset from the camera to the end effector  
        
        

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


        # #offset = np.array([0.0, 56.39, -3.05]) / 1000
        #depth_offset = np.array([27.50, 00.00, 00.00]) / 1000

       # world_coords = world_coords + depth_offset
        # print("adjusted world coords: ", world_coords + offset)

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
        #robot_frame_coords = robot_frame_coords + offset

        # # Publish marker at the transformed point
        # if robot_frame_coords is not None:
        #     self.publish_marker(robot_frame_coords)

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

        offset = np.array([0.00, 66.00, 0.00]) / 1000
        world_coords[0] += offset[0]
        world_coords[1] += offset[1]
        world_coords[2] += offset[2]

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

            # print("Quaternion: ", rot)
            # axis, angle = self.quaternion_to_axis_angle(rot)
            # print("Axis: ", axis)
            # print("Angle (radians): ", angle)
            euler_angles = tf.transformations.euler_from_quaternion(rot)

            # Pretzel orientation
            quaternion_pretzels = tf.transformations.quaternion_from_euler(degrees(euler_angles[0]), degrees(euler_angles[1]), degrees(self.pretzel_angle))

            #Pretzel orientation
            #euler_angles[0] = self.pretzel_angle

            print("Euler Angles: ", degrees(euler_angles[0]), degrees(euler_angles[1]), degrees(euler_angles[2]))

            #Publish the robot point and euler angles
            # offset = np.array([56.39, 0.00, 0.00]) / 1000
            # robot_point[0][3] -= offset[0]
            # robot_point[1][3] += offset[1]
            # robot_point[2][3] += offset[2]


            from geometry_msgs.msg import PointStamped

            object_location_point = PointStamped()
            object_location_point.header.frame_id = "base_link"
            object_location_point.point.x = robot_point[0][3]
            object_location_point.point.y = robot_point[1][3]
            object_location_point.point.z = robot_point[2][3]

            

            #tool_frame_coords = self.transform_frames("/tool_frame", "/base_link", object_location_point)

            tool_frame_coords = self.listener.transformPoint("tool_frame", object_location_point)

            offset = np.array([56.39, 0.00, 35]) / 1000
            tool_frame_coords.point.x += offset[0]
            tool_frame_coords.point.y += offset[1]
            tool_frame_coords.point.z -= offset[2]

            if tool_frame_coords is not None:
                rospy.loginfo(f"tool_frame_coords : x={tool_frame_coords.point.x}, y={tool_frame_coords.point.y}, z={tool_frame_coords.point.z}," )
                base_frame_coords = tool_frame_coords = self.listener.transformPoint("base_link", tool_frame_coords)

            if base_frame_coords is not None:
                rospy.loginfo(f"base_frame_coords : x={base_frame_coords.point.x}, y={base_frame_coords.point.y}, z={base_frame_coords.point.z}," )


            
            object_location = "x: " + str(base_frame_coords.point.x) + " y: " + str(base_frame_coords.point.y) + " z: " + str(base_frame_coords.point.z) + " roll: " + str(degrees(euler_angles[0])) + " pitch: " + str(degrees(euler_angles[1])) + " yaw: " + str(degrees(self.pretzel_angle))
            rospy.loginfo("Publishing object location: %s", object_location)
            pub.publish(object_location)

            # Publish marker at the transformed point
            if base_frame_coords is not None:
                self.publish_marker(base_frame_coords, quaternion_pretzels)

            
            rospy.sleep(1)

            

            
            return robot_point[:3, 3]  # extract the (x, y, z) coordinates
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("Transform lookup failed")
            return None
        
    def transform_frames(self, target, source, coordinates):
        """
        Transform coordinates from the source frame to the target frame.
        """
        self.listener.waitForTransform(target, source, rospy.Time(0), rospy.Duration(4.0))
        
         # Create a TransformStamped object with the world coordinates
        coords_point = tf.transformations.translation_matrix(coordinates)
        coords_point[3][3] = 1.0  # homogeneous coordinate
            
        try:
            (trans, rot) = self.listener.lookupTransform(target, source, rospy.Time(0))
            
            transform = tf.transformations.concatenate_matrices(
                tf.transformations.translation_matrix(trans),
                tf.transformations.quaternion_matrix(rot)
            )

            frame_point = np.dot(transform, coords_point)

            euler_angles = tf.transformations.euler_from_quaternion(rot)

            rospy.sleep(1)
            return frame_point[:3, 3]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("Transform lookup failed")
            return None
    
    def publish_marker(self, robot_point, rot):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "robot_frame"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = robot_point.point.x
        marker.pose.position.y = robot_point.point.y
        marker.pose.position.z = robot_point.point.z
        marker.pose.orientation.x = rot[0]
        marker.pose.orientation.y = rot[1]
        marker.pose.orientation.z = rot[2]
        marker.pose.orientation.w = rot[3]
        marker.scale.x = 0.01  # Size of the sphere
        marker.scale.y = 0.01
        marker.scale.z = 0.01
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
