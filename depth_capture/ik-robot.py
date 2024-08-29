import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler

# Initialize moveit_commander and rospy
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('kinova_gen3_ik', anonymous=True)

# Initialize the robot and group
robot = moveit_commander.RobotCommander("robot_description")
scene = moveit_commander.PlanningSceneInterface("my_gen3")
group_name = "arm"  # Modify based on your move group name
group = moveit_commander.MoveGroupCommander(group_name, "my_gen3")

# Set the target position and orientation
target_pose = geometry_msgs.msg.Pose()
target_pose.position.x = 0.35936804
target_pose.position.y = -0.25516251
target_pose.position.z = 0.1124
target_pose.orientation.x = 0.47271326728671303
target_pose.orientation.y = -0.76862840011877
target_pose.orientation.z = 0.37973859491651496
target_pose.orientation.w = -0.20384098948154156

# Set the pose target
group.set_pose_target(target_pose)

# Compute the IK solution
plan = group.plan()
if plan:
    print("Found IK solution:")
    for joint in plan.joint_trajectory.points[0].positions:
        print(joint)
else:
    print("No IK solution found.")

# Shut down moveit_commander
moveit_commander.roscpp_shutdown()
