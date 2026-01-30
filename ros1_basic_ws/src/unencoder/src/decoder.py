#!/usr/bin/env python3
# 24年省赛二维码解码节点
import cv2
import numpy as np
import tf
from pyzbar import pyzbar
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from geometry_msgs.msg import Point
from std_msgs.msg import UInt8
from unencoder.msg import decode
from std_msgs.msg import Float32MultiArray

class QRDecoder:
    def __init__(self):
        self.bridge = CvBridge()
        self.tf_listener = tf.TransformListener()
        self.tf_source_frame = rospy.get_param("~tf_source_frame", "/map")
        self.tf_target_frame = rospy.get_param("~tf_target_frame", "/laser_link")
        # 发布二维码数据的话题
        self.qr_data_pub = rospy.Publisher("/qr_code_data", decode, queue_size=10)
        # 发布二维码像素坐标的话题
        self.qr_position_pub = rospy.Publisher("/qr_code_position", Point, queue_size=10)
        # 添加订阅者订阅/target_position话题
        self.tar_position_sub = rospy.Subscriber("/target_position", Float32MultiArray, self.target_callback)
        # 初始化tar_position变量
        self.tar_position = Float32MultiArray()
        # 记录已识别过的二维码内容，避免重复日志
        self.detected_qr_codes = set()
    
    def target_callback(self, data):
        # 更新tar_position变量
        self.tar_position = data
        
    def decode_qr_codes(self, image):
        # 获取 TF 坐标并显示
        try:
            self.tf_listener.waitForTransform(
                self.tf_source_frame,
                self.tf_target_frame,
                rospy.Time(0),
                rospy.Duration(0.05)
            )
            (trans, rot) = self.tf_listener.lookupTransform(
                self.tf_source_frame,
                self.tf_target_frame,
                rospy.Time(0)
            )
            tf_text = "Pos: ({:.2f}, {:.2f}, {:.2f})".format(trans[0], trans[1], trans[2])
            cv2.putText(image, tf_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf.Exception):
            rospy.logwarn_throttle(2.0, f"TF获取失败: {self.tf_source_frame} -> {self.tf_target_frame}")

        # 使用pyzbar解码二维码
        decoded_objects = pyzbar.decode(image)
        current_error_text = None

        for obj in decoded_objects:
            # 提取二维码数据
            data = obj.data.decode("utf-8")
            
            # 判断是否为首次识别到该二维码
            is_first_detection = data not in self.detected_qr_codes
            if is_first_detection:
                self.detected_qr_codes.add(data)
                rospy.loginfo(f"[首次识别] 二维码内容: '{data}'")
            # print("QR Code Data: ", data)
            
            # 发布二维码数据到话题，转换为decode
            qr_data_msg = decode()
            # 将字符串数据转换为数字发布
            try:
                # 如果整个数据是数字，则直接转换
                qr_data_msg.data = int(data) if data.isdigit() else ord(data[0]) if data else 0
                qr_data_msg.is_valid = True
            except ValueError:
                # 如果转换失败，则使用第一个字符的ASCII码值
                qr_data_msg.data = ord(data[0]) if data else 0
                if is_first_detection:
                    rospy.logwarn(f"数据转换异常，使用首字符ASCII: {qr_data_msg.data}")
            
            if is_first_detection:
                rospy.loginfo(f"发布QR数据: val={qr_data_msg.data}, valid={qr_data_msg.is_valid}")
            self.qr_data_pub.publish(qr_data_msg)
            
            # 发布二维码中心坐标到话题
            center_x = obj.rect.left + obj.rect.width / 2
            center_y = obj.rect.top + obj.rect.height / 2
            qr_position_msg = Point()
            qr_position_msg.x = center_x
            qr_position_msg.y = center_y
            qr_position_msg.z = 0  # z轴设为0，因为我们处理的是2D图像
            
            # 坐标持续发布（用于PID控制），但日志仅首次输出
            if is_first_detection:
                rospy.loginfo(f"发布QR中心坐标: ({center_x}, {center_y})")
            self.qr_position_pub.publish(qr_position_msg)
            
            # 打印二维码像素坐标
            # print("QR Code Position: ({}, {})".format(center_x, center_y))
            
            # 获取二维码位置用于绘制边界
            points = obj.polygon
            if len(points) > 4:
                hull = cv2.convexHull(np.array([point for point in points], dtype=np.float32))
                hull = list(map(tuple, np.squeeze(hull)))
            else:
                hull = points
            
            # 绘制二维码边界
            n = len(hull)
            for j in range(0, n):
                cv2.line(image, hull[j], hull[(j+1) % n], (255, 0, 0), 3)
            
            # 在图像上显示二维码数据
            x = obj.rect.left
            y = obj.rect.top
            cv2.putText(image, data, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            # 显示二维码中心坐标
            center_x = obj.rect.left + obj.rect.width / 2
            center_y = obj.rect.top + obj.rect.height / 2
            position_text = f"({int(center_x)}, {int(center_y)})"
            cv2.putText(image, position_text, (int(center_x), int(center_y) - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

            # 计算并记录当前误差（像素）用于实时显示
            err_x = center_x - 380
            err_y = center_y - 170
            err_mag = np.sqrt(err_x * err_x + err_y * err_y)
            current_error_text = f"Err(px): dx={err_x:.1f} dy={err_y:.1f} mag={err_mag:.1f}"
        
        # 绘制中心点和误差圆
        center_point = (380, 170)
        align_error = float(rospy.get_param('qr_align_error', 100))
        
        # 中心点（红色激光点）
        cv2.circle(image, center_point, 3, (0, 0, 255), -1)
        # 误差圆
        cv2.circle(image, center_point, int(align_error), (0, 255, 0), 2)
        # 文字标注
        cv2.putText(image, f"Error: {align_error:.1f}", (center_point[0] + int(align_error) + 5, center_point[1]), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # 若检测到二维码，显示当前误差
        if current_error_text:
            cv2.putText(image, current_error_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        return image
    
def main():
    # 初始化ros节点
    rospy.init_node('qr_decoder', anonymous=True)
    # 初始化类
    qr_decoder = QRDecoder()
    try:
        #打开摄像头
        cap = cv2.VideoCapture("/dev/video0")
        #判断是否成功打开摄像头
        if not cap.isOpened():
            print("无法打开摄像头")
            exit(-1)
        
        # 持续读取和处理视频流
        while not rospy.is_shutdown():
            #读取摄像头一帧
            ret , frame = cap.read()
            if not ret:
                print("无法获取帧")
                break
                
            #将图像转化为rgb格式
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            #调用解码二维码函数
            processed_frame = qr_decoder.decode_qr_codes(frame_rgb.copy())
            
            # 将处理后的图像转换回BGR格式用于显示
            processed_frame_bgr = cv2.cvtColor(processed_frame, cv2.COLOR_RGB2BGR)
            
            #输出视频
            cv2.imshow("QR Code Decoder", processed_frame_bgr)
            
            # 按ESC键退出
            if cv2.waitKey(1) & 0xFF == 27:
                break
                
        # 释放摄像头资源
        cap.release()
        cv2.destroyAllWindows()
        
    except KeyboardInterrupt:
        print("Shutting down")
        # 释放摄像头资源
        cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()