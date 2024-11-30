#!/opt/anaconda3/bin/python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from std_msgs.msg import Float32MultiArray
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import threading
import yaml
import calibrateCamera



# 获取 ArUco 字典
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
aruco_params = cv2.aruco.DetectorParameters()
aruco_length = 14 #10cm


def detect_aruco_markers(image):

    max_area = 0
    idindex = -1
    idtvec = np.zeros(3)

    # 转换为灰度图
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # 检测 ArUco 码
    corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
    
    
    
    if ids is not None and len(corners) > 0:
        
        area = np.zeros(len(ids))

        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, aruco_length, camera_matrix, dist_coeffs)
        
        
        for i in range(len(ids)):
            # 获取当前 ArUco 码的角点
            corner_points = corners[i][0]            

            corner_points = corner_points.reshape(-1, 2)  # 转换为 (4, 2) 形状
            area[i] = cv2.contourArea(corner_points)
            if area[i] > max_area:
                max_area = area[i]
                idindex = ids[i][0]
                idtvec = tvec[i]
   
        cv2.aruco.drawDetectedMarkers(image, corners, ids)
        
        rospy.loginfo(f"ArUco ID: {idindex}")
        rospy.loginfo(f"ArUco TVec: {idtvec}")
        
    else:
        pass

    return image,idindex,idtvec


def camera_config():
    
    yaml_file_path = calibrateCamera.yaml_file_path("ost.yaml")
    
    with open(yaml_file_path) as file:
        camera_config = yaml.safe_load(file)  
        
    mtx = np.array(camera_config["camera_matrix"]["data"], dtype=np.float32).reshape((3, 3))
    dist = np.array(camera_config["distortion_coefficients"]["data"])


    return mtx, dist



class ArUco_Detection:
    
    def __init__(self):
        
        #相机标定
        global camera_matrix,dist_coeffs
        camera_matrix,dist_coeffs = camera_config()
        
        
        # 初始化ROS节点
        rospy.init_node("ArUco_Detection")

        # 创建CvBridge实例
        self.bridge = CvBridge()
        
        # 创建发布者
        self.id_pub = rospy.Publisher("/ArUco_code/ID", Float64, queue_size=10)
        self.tvec_pub = rospy.Publisher("/ArUco_code/Tvec", Float32MultiArray, queue_size=10)
        self.tvec_msg = Float32MultiArray()

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
        cv_image_with_markers, aruco_ids,aruco_tvec = detect_aruco_markers(cv_image)

        # 发布检测到的 ArUco ID
        if aruco_ids is not None:
            self.id_pub.publish(aruco_ids)
            
            
            
            self.tvec_msg.data = aruco_tvec.flatten().tolist()            
            self.tvec_pub.publish(self.tvec_msg)
            
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
        ArUco_Detection()
    except rospy.ROSInterruptException:
        pass