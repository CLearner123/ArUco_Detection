#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import threading


# 获取 ArUco 字典
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
aruco_params = cv2.aruco.DetectorParameters()

# 全局变量
# cv_image = None
# bridge = CvBridge()

def detect_aruco_markers(image):

    max_area = 0
    idindex = -1


    # 转换为灰度图
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # 检测 ArUco 码
    corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
    
    
    
    if ids is not None and len(corners) > 0:
        
        area = np.zeros(len(ids))
        # 绘制检测到的 ArUco 码边框
        # cv2.aruco.drawDetectedMarkers(image, corners, ids)
        
        for i in range(len(ids)):
            # 获取当前 ArUco 码的角点
            corner_points = corners[i][0]            
            # 检查是否为空并将角点重塑为 (4, 2) 的形状
            # if corner_points is not None and corner_points.shape[0] >= 3:
            corner_points = corner_points.reshape(-1, 2)  # 转换为 (4, 2) 形状
            area[i] = cv2.contourArea(corner_points)
            if area[i] > max_area:
                max_area = area[i]
                idindex = ids[i][0]
                
   
        cv2.aruco.drawDetectedMarkers(image, corners, ids)
        
        rospy.loginfo(f"Detected ArUco ID: {idindex}")

        
    else:
        pass

    return image,idindex


class QRcode_Detection:
    
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node("QRcode_Detection")

        # 创建CvBridge实例
        self.bridge = CvBridge()
        
        # 创建发布者
        self.self_pub = rospy.Publisher("/QRcode_ID", Float64, queue_size=10)

        try:
            # 创建和启动线程
            pub_thread = threading.Thread(target=self.publisher)
            sub_thread = threading.Thread(target=self.subscriber)
            
            pub_thread.start()
            sub_thread.start()
            
            pub_thread.join()
            sub_thread.join()
        
        except rospy.ROSException as e:
            rospy.logerr(f"ROS Exception: {e}")

    # ArUco检测的回调函数
    def arucoCallback(self, msg):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"Error converting ROS Image to OpenCV format: {e}")
            return

        # 检测 ArUco 码并标记图像
        cv_image_with_markers, aruco_ids = detect_aruco_markers(cv_image)

        # 发布检测到的 ArUco ID
        if aruco_ids is not None:
            self.self_pub.publish(aruco_ids)
        
        # 显示图像
        cv2.imshow("Camera Image with ArUco Detection", cv_image_with_markers)

        # 按 'q' 退出
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            rospy.signal_shutdown("User pressed 'q' to quit.")
            rospy.loginfo("Shutting down by user request.")
    
    # 订阅者函数
    def subscriber(self):
        rospy.Subscriber("/camera/color/image_raw", Image, self.arucoCallback, queue_size=10)
        rospy.spin()
    
    # 发布者函数
    def publisher(self):
        # 发布者的行为会在 arucoCallback 中执行，所以这里可能不需要额外的代码
        rospy.loginfo("Publisher thread started")



if __name__ == '__main__':
    try:
        QRcode_Detection()
    except rospy.ROSInterruptException:
        pass