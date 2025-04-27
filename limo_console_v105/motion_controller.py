#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
运动控制模块
用于LIMO控制台中的运动控制功能，发布ROS速度话题
基于teleop_twist_keyboard.py的实现，提供更稳定的速度发布
"""

import rospy
import threading
import time
import os
import subprocess
from geometry_msgs.msg import Twist

class PublishThread(threading.Thread):
    """速度发布线程，基于teleop_twist_keyboard.py的实现"""
    
    def __init__(self, topic_name="/cmd_vel", rate=10):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher(topic_name, Twist, queue_size=1)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.speed = 0.5  # 默认线速度系数
        self.turn = 1.0   # 默认角速度系数
        self.condition = threading.Condition()
        self.done = False
        self.publish_count = 0
        self.error_count = 0
        self.last_publish_time = 0
        
        # 设置超时时间
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None
        
        # 设置为守护线程
        self.daemon = True
    
    def wait_for_subscribers(self, timeout_sec=2.0):
        """等待订阅者连接，带有超时机制"""
        start_time = time.time()
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            # 检查是否超时
            if time.time() - start_time > timeout_sec:
                print(f"等待订阅者超时，继续执行（无订阅者连接到 {self.publisher.name}）")
                return False
                
            if i == 4:
                print(f"等待订阅者连接到 {self.publisher.name}")
            rospy.sleep(0.2)  # 缩短睡眠时间，提高响应性
            i += 1
            i = i % 5
        
        if rospy.is_shutdown():
            raise Exception("在订阅者连接前收到关闭请求")
            
        print(f"已有 {self.publisher.get_num_connections()} 个订阅者连接到 {self.publisher.name}")
        return True
    
    def update(self, x, y, z, th, speed, turn):
        """更新速度值"""
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.th = th
        self.speed = speed
        self.turn = turn
        # 通知发布线程有新消息
        self.condition.notify()
        self.condition.release()
    
    def stop(self):
        """停止发布线程"""
        self.done = True
        self.update(0, 0, 0, 0, 0, 0)
        self.join()
    
    def run(self):
        """线程运行函数"""
        twist = Twist()
        
        while not self.done and not rospy.is_shutdown():
            # 获取锁并等待新消息或超时
            self.condition.acquire()
            self.condition.wait(self.timeout)
            
            # 复制状态到Twist消息
            twist.linear.x = self.x * self.speed
            twist.linear.y = self.y * self.speed
            twist.linear.z = self.z * self.speed
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = self.th * self.turn
            
            self.condition.release()
            
            # 发布消息
            try:
                self.publisher.publish(twist)
                self.publish_count += 1
                self.last_publish_time = time.time()
                
                # 每50次发布打印一次日志
                if self.publish_count % 50 == 0:
                    print(f"已发布速度命令 {self.publish_count} 次: linear_x={twist.linear.x:.2f}, linear_y={twist.linear.y:.2f}, angular_z={twist.angular.z:.2f}")
            except Exception as e:
                self.error_count += 1
                print(f"发布速度命令失败 ({self.error_count}): {str(e)}")
        
        # 线程退出时发布停止消息
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        
        try:
            for i in range(3):  # 发布多次以确保停止
                self.publisher.publish(twist)
                time.sleep(0.1)
            print("已发送停止命令")
        except Exception as e:
            print(f"发布停止命令失败: {str(e)}")

class MotionController(object):
    """运动控制类，负责发布ROS速度话题"""
    
    def __init__(self):
        # ROS相关变量
        self.topic_name = "/cmd_vel"
        self.publish_rate = 10  # Hz
        self.is_active = False
        self.node_initialized = False
        
        # 发布线程
        self.pub_thread = None
        
        # 速度相关变量
        self.linear_x = 0.0  # 前后速度
        self.linear_y = 0.0  # 左右速度（麦轮模式）
        self.angular_z = 0.0  # 旋转速度
        
        # 速度限制
        self.max_linear_speed = 0.5  # m/s
        self.max_angular_speed = 1.0  # rad/s
    
    def start(self):
        """启动运动控制"""
        if self.is_active and self.pub_thread and not self.pub_thread.done:
            print("运动控制已经处于激活状态")
            return True
        
        try:
            # 检查ROS核心是否运行
            if not self._check_ros_running():
                print("启动运动控制失败：ROS核心未运行")
                return False
            
            # 初始化ROS节点（如果尚未初始化）
            if not self._initialize_ros_node():
                print("启动运动控制失败：无法初始化ROS节点")
                return False
            
            # 创建并启动发布线程
            try:
                self.pub_thread = PublishThread(self.topic_name, self.publish_rate)
                self.pub_thread.start()
                
                # 等待订阅者连接
                try:
                    self.pub_thread.wait_for_subscribers(timeout_sec=2.0)

                except Exception as e:
                    print(f"等待订阅者时出错: {str(e)}")
                
                # 发送初始零速度命令
                self.pub_thread.update(0, 0, 0, 0, self.max_linear_speed, self.max_angular_speed)
                
                self.is_active = True
                print("运动控制已成功启动")
                return True
            except Exception as e:
                print(f"创建发布线程时出错: {str(e)}")
                return False
        
        except Exception as e:
            print(f"启动运动控制时出错: {str(e)}")
            self.is_active = False
            return False
    
    def stop(self):
        """停止运动控制"""
        if not self.is_active:
            return
        
        # 停止发布线程
        if self.pub_thread:
            try:
                self.pub_thread.stop()
                print(f"运动控制已停止，共发布 {self.pub_thread.publish_count} 次命令，发生 {self.pub_thread.error_count} 次错误")
            except Exception as e:
                print(f"停止发布线程时出错: {str(e)}")
        
        # 关闭发布者
        if hasattr(self.pub_thread, 'publisher'):
            try:
                self.pub_thread.publisher.unregister()
                print("已关闭发布者")
            except:
                pass
        
        self.is_active = False
        self.node_initialized = False  # 重置节点初始化状态，下次启动时将重新初始化

    
    def _check_ros_running(self):
        """检查ROS核心是否运行"""
        try:
            # 方法1: 使用rostopic list命令
            result = subprocess.run(
                ["rostopic", "list"], 
                stdout=subprocess.PIPE, 
                stderr=subprocess.PIPE, 
                timeout=1
            )
            if result.returncode == 0:
                return True
            
            # 方法2: 检查ROS_MASTER_URI环境变量
            if 'ROS_MASTER_URI' in os.environ:
                try:
                    import xmlrpc.client
                    master_uri = os.environ['ROS_MASTER_URI']
                    master = xmlrpc.client.ServerProxy(master_uri)
                    # 尝试调用一个简单的API
                    master.getSystemState('/rostopic')
                    return True
                except:
                    pass
            
            return False
        except Exception as e:
            print(f"检查ROS状态时出错: {str(e)}")
            return False
    
    def _initialize_ros_node(self):
        """初始化ROS节点"""
        if self.node_initialized:
            return True
            
        try:
            # 检查是否已经有节点在运行
            if rospy.get_node_uri():
                print("ROS节点已经初始化")
                self.node_initialized = True
                return True
            
            # 初始化新节点
            rospy.init_node('limo_motion_controller', anonymous=True, disable_signals=True)
            self.node_initialized = True
            print("已初始化ROS节点: limo_motion_controller")
            return True
        except Exception as e:
            print(f"初始化ROS节点时出错: {str(e)}")
            return False
    
    def set_velocity(self, linear_x, linear_y, angular_z):
        """设置速度值"""
        if not self.is_active or not self.pub_thread:
            print("运动控制未激活，无法设置速度")
            return
        
        # 限制速度在安全范围内
        linear_x = max(-self.max_linear_speed, min(linear_x, self.max_linear_speed))
        linear_y = max(-self.max_linear_speed, min(linear_y, self.max_linear_speed))
        angular_z = max(-self.max_angular_speed, min(angular_z, self.max_angular_speed))
        
        # 计算标准化的速度值
        x_val = linear_x / self.max_linear_speed if self.max_linear_speed > 0 else 0
        y_val = linear_y / self.max_linear_speed if self.max_linear_speed > 0 else 0
        th_val = angular_z / self.max_angular_speed if self.max_angular_speed > 0 else 0
        
        # 更新发布线程中的速度值
        self.pub_thread.update(x_val, y_val, 0, th_val, self.max_linear_speed, self.max_angular_speed)
    
    def move_forward(self, speed=0.2):
        """向前移动"""
        self.set_velocity(speed, 0.0, 0.0)
        print(f"向前移动: {speed}")
    
    def move_backward(self, speed=0.2):
        """向后移动"""
        self.set_velocity(-speed, 0.0, 0.0)
        print(f"向后移动: {speed}")
    
    def turn_left(self, speed=0.5):
        """向左转"""
        self.set_velocity(0.0, 0.0, speed)
        print(f"向左转: {speed}")
    
    def turn_right(self, speed=0.5):
        """向右转"""
        self.set_velocity(0.0, 0.0, -speed)
        print(f"向右转: {speed}")
    
    def move_left(self, speed=0.2):
        """向左移动（麦轮模式）"""
        self.set_velocity(0.0, speed, 0.0)
        print(f"向左移动: {speed}")
    
    def move_right(self, speed=0.2):
        """向右移动（麦轮模式）"""
        self.set_velocity(0.0, -speed, 0.0)
        print(f"向右移动: {speed}")
    
    def stop_motion(self):
        """停止移动"""
        if self.is_active and self.pub_thread:
            self.set_velocity(0.0, 0.0, 0.0)
            print("停止移动")
    
    def set_from_joystick(self, x_ratio, y_ratio):
        """根据摇杆位置设置速度
        
        参数:
            x_ratio: 范围[-1, 1]，负值表示左，正值表示右
            y_ratio: 范围[-1, 1]，负值表示后，正值表示前
        """
        # 计算线速度和角速度
        linear_x = y_ratio * self.max_linear_speed
        angular_z = -x_ratio * self.max_angular_speed  # 负号使得右转为负值
        
        # 设置速度
        self.set_velocity(linear_x, 0.0, angular_z)
