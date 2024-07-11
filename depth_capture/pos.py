import cv2
import numpy as np

# Function to capture RGB and Depth images from your depth camera
def capture_images():
    # Replace this with the actual code to capture from your camera
    rgb_image = cv2.imread("rgb_image.jpg")
    depth_image = cv2.imread("depth_image.png", cv2.IMREAD_UNCHANGED)  # Depth image in meters
    return rgb_image, depth_image

rgb_image, depth_image = capture_images()

# Example coordinates of the point of interest in the RGB image
point_x = 250
point_y = 300

depth_value = depth_image[point_y, point_x]  # Depth value in meters

# Camera intrinsic parameters (example values, replace with your camera's actual parameters)
fx = 600  # Focal length in x
fy = 600  # Focal length in y
cx = rgb_image.shape[1] / 2  # Principal point x
cy = rgb_image.shape[0] / 2  # Principal point y

# Calculate real-world coordinates
real_x = (point_x - cx) * depth_value / fx
real_y = (point_y - cy) * depth_value / fy
real_z = depth_value

print(f"Real-world position: X={real_x}, Y={real_y}, Z={real_z}")
