#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

# 获取 ArUco 字典
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
aruco_params = cv2.aruco.DetectorParameters()

# 全局变量
cv_image = None
bridge = CvBridge()

def detect_aruco_markers(image):
    """
    在图像中检测 ArUco 码并标记出来
    """
    # 转换为灰度图
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # 检测 ArUco 码
    corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

    if ids is not None:
        # 绘制检测到的 ArUco 码边框
        cv2.aruco.drawDetectedMarkers(image, corners, ids)

        # 打印检测到的 ID
        for marker_id in ids.flatten():
            rospy.loginfo(f"Detected ArUco ID: {marker_id}")
    else:
        rospy.loginfo("No ArUco markers detected.")

    return image

def ImageCallback(msg):
    global cv_image
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    except CvBridgeError as e:
        rospy.logerr(f"Error converting ROS Image to OpenCV format: {e}")
        return

    # 检测 ArUco 码并标记图像
    cv_image_with_markers = detect_aruco_markers(cv_image)

    # 显示图像
    cv2.imshow("Camera Image with ArUco Detection", cv_image_with_markers)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        rospy.signal_shutdown("User pressed 'q' to quit.")
        rospy.loginfo("Shutting down by user request.")

if __name__ == "__main__":
    rospy.init_node("Image_sub")

    # 订阅图像话题
    rospy.Subscriber("/camera/color/image_raw", Image, ImageCallback, queue_size=10)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Node terminated by user.")
    finally:
        cv2.destroyAllWindows()
