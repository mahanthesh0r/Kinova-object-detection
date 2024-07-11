#!/usr/bin/env python3

import rospy
import tf
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def publish_marker(frame_id, point, marker_pub):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = "robot_frame"
    marker.id = 0
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.pose.position.x = 0.8 #10.64852893 -61.46935271 -54.43308728
    marker.pose.position.y = -0.3
    marker.pose.position.z = 0.05
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.08 # Size of the sphere
    marker.scale.y = 0.08
    marker.scale.z = 0.08
    marker.color.a = 1.0  # Alpha channel
    marker.color.r = 1.0  # Red
    marker.color.g = 0.0  # Green
    marker.color.b = 0.0  # Blue

    marker_pub.publish(marker)

def listener():
    rospy.init_node('tf_listener', anonymous=True)
    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)  # 10 Hz
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform('/base_link', '/camera_link', rospy.Time(0))
            rospy.loginfo("Translation: {}".format(trans))
            rospy.loginfo("Rotation: {}".format(rot))

            # Publish marker at the transformed point
            publish_marker('base_link', trans, marker_pub)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("Transform lookup failed")

        rate.sleep()

if __name__ == '__main__':
    listener()
