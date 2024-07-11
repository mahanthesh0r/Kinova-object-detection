import rospy 
import cv2
import numpy as np
from sensor_msgs.msg import Image
import threading
from cv_bridge import CvBridge
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.messages import Session_pb2, Base_pb2
from kortex_api.TCPTransport import TCPTransport
from kortex_api.RouterClient import RouterClient
from kortex_api.SessionManager import SessionManager


# Create closure to set an event after an END or an ABORT
def check_for_end_or_abort(e):
    """Return a closure checking for END or ABORT notifications

    Arguments:
    e -- event to signal when the action is completed
        (will be set when an END or ABORT occurs)
    """
    def check(notification, e = e):
        print("EVENT : " + \
              Base_pb2.ActionEvent.Name(notification.action_event))
        if notification.action_event == Base_pb2.ACTION_END \
        or notification.action_event == Base_pb2.ACTION_ABORT:
            e.set()
    return check

def error_callback(k_error):
    print(f"Error occurred: {k_error}")

def connect_to_arm():
    transport = TCPTransport()
    router = RouterClient(transport, error_callback)
    transport.connect('192.168.1.10', 10000)  # Replace with your arm's IP and port

    session_info = Session_pb2.CreateSessionInfo()
    session_info.username = 'admin'
    session_info.password = 'admin'
    session_manager = SessionManager(router)
    session_manager.CreateSession(session_info)

    base = BaseClient(router)
    base_cyclic = BaseCyclicClient(router)
    return base

def move_to_depth(base, depth):
    command = Base_pb2.Action()
    command.name = "move to depth"
    command.application_data = ""
    
    
    # pose = Base_pb2.ConstrainedPose()
    pose = command.reach_pose.target_pose
    pose.x = 0.337
    pose.y = -0.001
    pose.z = 0.444 - depth
    pose.theta_x = 179.1
    pose.theta_y = 5.5
    pose.theta_z = 93.3

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    print("Executing Actions")

    
    # command.reach_pose.target_pose.CopyFrom(pose.target_pose)


    base.ExecuteAction(command)
   
    finished = e.wait(20)
  
    base.Unsubscribe(notification_handle)

    if finished:
        print("Safe Position reached")
    else:
        print("Timeout")
    

class DepthCamera:
    def __init__(self):
        self.bridge = CvBridge()
        self.depth_image = None
        rospy.init_node('depth_camera_node', anonymous=True)
        rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_callback)

    def depth_callback(self, data):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        except CvBridgeError as e:
            print(e)

    def get_depth_of_object(self, x, y):
        if self.depth_image is not None:
            return self.depth_image[y, x]
        return None
    
if __name__ == "__main__":
    dc = DepthCamera()
    rospy.sleep(1)

    object_x = 50
    object_y = 175

    depth = dc.get_depth_of_object(object_x, object_y)
    depth_gripper_obj = depth - 23
    depth_final = depth_gripper_obj/100
    if depth is not None:
        
        print(f"Depth of object at ({object_x}, {object_y}): gripper to obj {depth_gripper_obj} Cm, Final depth {depth_final} m")

        base = connect_to_arm()
        move_to_depth(base, depth_final)
        rospy.sleep(2)

    else:
        print("depth data not available")