import torch 
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from yolov5 import detect

class CameraViewer:
    def __init__(self):
        rospy.init_node('camera_viewer', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        cv2.imshow("Camera Feed", cv_image)
        cv2.waitKey(1)
    
    def run(self):
        rospy.spin()

    
if __name__ == '__main__':
    camera_viewer = CameraViewer()
    model = torch.hub.load('ultralytics/yolov5','yolov5s', pretrained = True)

    results = model(camera_viewer.run())