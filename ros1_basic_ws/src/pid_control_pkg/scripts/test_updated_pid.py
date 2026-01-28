#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试更新后的PID控制器 - 新的消息类型和单位
- /target_position: std_msgs/Float32MultiArray [x_cm, y_cm, z_cm, yaw_deg]
- /height: std_msgs/Int16 (cm)
- /target_velocity: std_msgs/Float32MultiArray [vx_cm/s, vy_cm/s, vz_cm/s, vyaw_deg/s]
"""

import rospy
import std_msgs.msg
import tf
from math import sin, cos, pi, sqrt

def test_updated_pid_controller():
    """测试更新后的PID控制器"""
    rospy.init_node('test_updated_pid', anonymous=True)
    
    # 发布器
    target_position_pub = rospy.Publisher('/target_position', std_msgs.msg.Float32MultiArray, queue_size=1)
    height_pub = rospy.Publisher('/height', std_msgs.msg.Int16, queue_size=1)
    
    # TF广播器（模拟map->laser_link变换，TF中的位置单位仍为m）
    tf_broadcaster = tf.TransformBroadcaster()
    
    # 订阅器（监听输出速度）
    velocity_data = {'vx_cm': 0, 'vy_cm': 0, 'vz_cm': 0, 'vyaw_deg': 0}
    
    def velocity_callback(msg):
        if len(msg.data) >= 4:
            velocity_data['vx_cm'] = msg.data[0]     # cm/s
            velocity_data['vy_cm'] = msg.data[1]     # cm/s  
            velocity_data['vz_cm'] = msg.data[2]     # cm/s
            velocity_data['vyaw_deg'] = msg.data[3]  # deg/s
            
            # 计算速度大小
            speed_xy_cm = sqrt(msg.data[0]**2 + msg.data[1]**2)
            
            rospy.loginfo("PID输出 - 线速度: [%.1f, %.1f, %.1f]cm/s 角速度: %.1f°/s 总速度: %.1fcm/s",
                         msg.data[0], msg.data[1], msg.data[2], msg.data[3], speed_xy_cm)
            
            # 检查速度限制是否合理
            if speed_xy_cm > 40.0:  # 超过40cm/s
                rospy.logwarn("线速度过高: %.1f cm/s", speed_xy_cm)
            if abs(msg.data[3]) > 35.0:  # 超过35度/s
                rospy.logwarn("角速度过高: %.1f deg/s", msg.data[3])
    
    velocity_sub = rospy.Subscriber('/target_velocity', std_msgs.msg.Float32MultiArray, velocity_callback)
    
    rate = rospy.Rate(50.0)  # 50Hz
    
    rospy.loginfo("=== 开始测试更新版PID控制器 ===")
    rospy.loginfo("新的消息类型和单位：")
    rospy.loginfo("  - /target_position: Float32MultiArray [x_cm, y_cm, z_cm, yaw_deg]")
    rospy.loginfo("  - /height: Int16 (cm)")  
    rospy.loginfo("  - /target_velocity: Float32MultiArray [vx_cm/s, vy_cm/s, vz_cm/s, vyaw_deg/s]")
    
    # 测试场景参数 (单位: cm, 度)
    scenarios = [
        {"name": "近距离精确定位", "target": (50.0, 30.0, 100.0, 20.0), "duration": 8},
        {"name": "中等距离移动", "target": (150.0, 100.0, 120.0, 45.0), "duration": 12}, 
        {"name": "大角度转向测试", "target": (100.0, 50.0, 100.0, 180.0), "duration": 10},
        {"name": "垂直运动测试", "target": (100.0, 50.0, 200.0, 90.0), "duration": 8},
        {"name": "组合运动测试", "target": (200.0, 150.0, 80.0, -45.0), "duration": 15},
    ]
    
    # 初始位置 (单位: cm, 度，但TF发布时转换为m, rad)
    current_x_cm, current_y_cm, current_z_cm, current_yaw_deg = 0.0, 0.0, 100.0, 0.0
    
    scenario_idx = 0
    scenario_start_time = rospy.Time.now()
    
    while not rospy.is_shutdown() and scenario_idx < len(scenarios):
        current_time = rospy.Time.now()
        dt = (current_time - scenario_start_time).to_sec()
        
        # 获取当前测试场景
        scenario = scenarios[scenario_idx]
        target_x_cm, target_y_cm, target_z_cm, target_yaw_deg = scenario["target"]
        
        # 模拟机器人运动（简单的响应模型）
        response_rate = 0.02  # 响应速率
        current_x_cm += response_rate * velocity_data['vx_cm']
        current_y_cm += response_rate * velocity_data['vy_cm'] 
        current_z_cm += response_rate * velocity_data['vz_cm']
        current_yaw_deg += response_rate * velocity_data['vyaw_deg']
        
        # 限制yaw在-180到180度范围
        while current_yaw_deg > 180.0: current_yaw_deg -= 360.0
        while current_yaw_deg < -180.0: current_yaw_deg += 360.0
        
        # 发布TF变换（转换为m和rad）
        current_x_m = current_x_cm / 100.0
        current_y_m = current_y_cm / 100.0 
        current_z_m = current_z_cm / 100.0
        current_yaw_rad = current_yaw_deg * pi / 180.0
        
        tf_broadcaster.sendTransform(
            (current_x_m, current_y_m, current_z_m),
            tf.transformations.quaternion_from_euler(0, 0, current_yaw_rad),
            current_time,
            "laser_link",
            "map"
        )
        
        # 发布目标位置 (std_msgs/Float32MultiArray)
        target_pos_msg = std_msgs.msg.Float32MultiArray()
        target_pos_msg.data = [target_x_cm, target_y_cm, target_z_cm, target_yaw_deg]
        target_position_pub.publish(target_pos_msg)
        
        # 发布当前高度 (std_msgs/Int16，单位cm)
        height_msg = std_msgs.msg.Int16()
        height_msg.data = int(current_z_cm)
        height_pub.publish(height_msg)
        
        # 计算误差 (单位: cm, 度)
        error_x_cm = target_x_cm - current_x_cm
        error_y_cm = target_y_cm - current_y_cm
        error_z_cm = target_z_cm - current_z_cm
        error_yaw_deg = target_yaw_deg - current_yaw_deg
        
        # 角度误差归一化
        while error_yaw_deg > 180.0: error_yaw_deg -= 360.0
        while error_yaw_deg < -180.0: error_yaw_deg += 360.0
        
        distance_xy_cm = sqrt(error_x_cm**2 + error_y_cm**2)
        
        # 每2秒打印一次状态
        if int(dt) % 2 == 0 and (dt - int(dt)) < 0.1:
            rospy.loginfo("=== 场景%d: %s (%.1fs/%.1fs) ===", 
                         scenario_idx + 1, scenario["name"], dt, scenario["duration"])
            rospy.loginfo("当前位置: [%.1f, %.1f, %.1f]cm, yaw=%.1f°", 
                         current_x_cm, current_y_cm, current_z_cm, current_yaw_deg)
            rospy.loginfo("目标位置: [%.1f, %.1f, %.1f]cm, yaw=%.1f°", 
                         target_x_cm, target_y_cm, target_z_cm, target_yaw_deg)
            rospy.loginfo("位置误差: [%.1f, %.1f, %.1f]cm, yaw=%.1f°, 距离=%.1fcm", 
                         error_x_cm, error_y_cm, error_z_cm, error_yaw_deg, distance_xy_cm)
            
            # 检查是否到达目标（容差: 6cm, 5度）
            if (abs(error_x_cm) <= 6.0 and abs(error_y_cm) <= 6.0 and 
                abs(error_z_cm) <= 6.0 and abs(error_yaw_deg) <= 5.0):
                rospy.loginfo("✓ 已到达目标位置！")
        
        # 切换到下一个场景
        if dt >= scenario["duration"]:
            rospy.loginfo("场景%d完成，切换到下一个场景", scenario_idx + 1)
            scenario_idx += 1
            scenario_start_time = current_time
        
        rate.sleep()
    
    rospy.loginfo("=== 所有测试场景完成 ===")

if __name__ == '__main__':
    try:
        test_updated_pid_controller()
    except rospy.ROSInterruptException:
        rospy.loginfo("测试中断")