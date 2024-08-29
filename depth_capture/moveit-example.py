#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose, PoseStamped
from tf.transformations import quaternion_from_euler

def talker():
    pub = rospy.Publisher('arm_pose', PoseStamped, queue_size=10)
    rospy.init_node('robot_arm_pose_publisher', anonymous=True)
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.header.stamp = rospy.Time.now()
        pose.header.seq = 1
        # Set position
        pose.pose.position.x = 0.39235062
        pose.pose.position.y = -0.2857303
        pose.pose.position.z = 0.0476794

        # Set orientation (as a quaternion)
        roll = 0.0
        pitch = 0.0
        yaw = 1.57
        quaternion = quaternion_from_euler(roll, pitch, yaw)
        pose.pose.orientation.x = 0.4782606926919657
        pose.pose.orientation.y = -0.7722499972611762
        pose.pose.orientation.z = 0.3695210560790377
        pose.pose.orientation.w = -0.19583370667520086

        rospy.loginfo(pose)
        pub.publish(pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
