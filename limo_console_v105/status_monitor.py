#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LIMO状态监控模块
负责监控LIMO底盘状态，支持pylimo库和ROS话题两种数据源
"""

import threading
import time
import subprocess
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Int32
from PyQt5.QtCore import QObject, pyqtSignal

# 检查pylimo库是否可用
LIMO_AVAILABLE = False
pylimo = None
limomsg = None
try:
    import pylimo
    from pylimo import limomsg
    LIMO_AVAILABLE = True
    print("pylimo库已成功导入")
except ImportError:
    LIMO_AVAILABLE = False
    print("警告: pylimo库未找到，无法使用直接读取功能")

# 导入数据模拟器
from data_simulator import DataSimulator

class StatusMonitor(QObject):
    """LIMO状态监控类，负责获取和更新LIMO底盘状态数据"""
    
    # 定义信号
    status_updated = pyqtSignal()
    
    def __init__(self, parent=None):
        super(StatusMonitor, self).__init__(parent)
        
        # 数据锁，用于线程安全
        self.data_lock = threading.Lock()
        
        # 状态数据
        self.battery_voltage = 0.0
        self.motion_mode = 0
        self.control_mode = 0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.lateral_velocity = 0.0
        self.steering_angle = 0.0
        self.error_code = 0
        self.vehicle_state = 0
        
        # 历史数据
        self.timestamps = []
        self.battery_history = []
        self.velocity_history = []
        self.max_history_size = 100
        
        # 运动模式字典
        self.motion_mode_dict = {
            0: "差速模式",
            1: "阿克曼模式",
            2: "麦轮模式",
            3: "全向模式"
        }
        
        # 控制模式字典
        self.control_mode_dict = {
            0: "遥控器模式",
            1: "自动驾驶模式"
        }
        
        # 错误代码字典
        self.error_code_dict = {
            0x0001: "电池电压低",
            0x0002: "IMU错误",
            0x0004: "里程计错误",
            0x0008: "电机驱动错误",
            0x0010: "通信超时",
            0x0020: "紧急停止"
        }
        
        # pylimo库相关变量
        self.pylimo_thread = None
        self.pylimo_running = False
        self.limo = None
        self.limo_connected = False
        
        # ROS话题相关变量
        self.ros_subscribers = []
        self.ros_running = False
        
        # 数据模拟器
        self.data_simulator = DataSimulator(self)
        self.simulation_running = False
        
        # 初始化状态
        self.initialized = False
        
        # 初始化
        self.initialize()
    
    def initialize(self):
        """初始化状态监控器"""
        if self.initialized:
            return
        
        print("初始化状态监控器...")
        
        # 尝试启动pylimo库数据读取
        if self.start_pylimo():
            print("使用pylimo库进行状态监控")
        else:
            # 如果pylimo库不可用，尝试启动ROS话题订阅
            try:
                self.start_ros_subscriber()
                print("使用ROS话题进行状态监控")
            except Exception as e:
                print(f"启动ROS话题订阅失败: {str(e)}")
                # 如果ROS话题也不可用，启动模拟数据
                self.start_simulation()
                print("使用模拟数据进行状态监控")
        
        self.initialized = True
    
    def stop_all_data_sources(self):
        """停止所有数据源"""
        # 停止pylimo库数据读取
        if self.pylimo_running:
            self.stop_pylimo()
        
        # 停止ROS话题订阅
        if self.ros_running:
            self.stop_ros_subscriber()
        
        # 停止模拟数据
        if self.simulation_running:
            self.stop_simulation()
    
    def start_simulation(self):
        """启动数据模拟"""
        if self.simulation_running:
            return
        
        print("启动数据模拟模式")
        self.simulation_running = True
        self.data_simulator.start()
    
    def stop_simulation(self):
        """停止数据模拟"""
        if not self.simulation_running:
            return
        
        print("停止数据模拟模式")
        self.simulation_running = False
        self.data_simulator.stop()
    
    def start_pylimo(self):
        """启动pylimo库数据读取"""
        global pylimo, limomsg, LIMO_AVAILABLE
        
        # 检查pylimo库是否可用
        if not LIMO_AVAILABLE:
            try:
                import pylimo
                from pylimo import limomsg
                LIMO_AVAILABLE = True
                print("pylimo库已成功导入")
            except ImportError:
                LIMO_AVAILABLE = False
                print("警告: pylimo库未找到，无法启动直接读取")
                # 启动模拟数据生成
                self.start_simulation()
                return False
        
        if self.pylimo_running:
            return True
        
        try:
            # 初始化LIMO设备
            # 尝试不同的串口设备路径
            device_paths = ["/dev/ttyTHS0", "/dev/ttyTHS1", "/dev/ttyUSB1"]
            connected = False
            
            for device_path in device_paths:
                try:
                    print(f"尝试连接LIMO设备: {device_path}")
                    self.limo = pylimo.LIMO(device_path)
                    # 确保limomsg模块已初始化
                    limomsg._init()
                    connected = True
                    print(f"成功连接到LIMO设备: {device_path}")
                    break
                except Exception as e:
                    print(f"连接到 {device_path} 失败: {str(e)}")
            
            if not connected:
                print("无法连接到任何LIMO设备，尝试默认初始化")
                self.limo = pylimo.LIMO()
                limomsg._init()
            
            self.limo_connected = True
            
            # 启动数据读取线程
            self.pylimo_running = True
            self.pylimo_thread = threading.Thread(target=self._pylimo_read_loop)
            self.pylimo_thread.daemon = True
            self.pylimo_thread.start()
            
            # 停止模拟数据
            self.stop_simulation()
            
            print("已启动pylimo库数据读取")
            return True
        except Exception as e:
            print(f"启动pylimo库数据读取时出错: {str(e)}")
            self.limo_connected = False
            self.pylimo_running = False
            
            # 启动模拟数据生成
            self.start_simulation()
            return False
    
    def stop_pylimo(self):
        """停止pylimo库数据读取"""
        self.pylimo_running = False
        
        # 等待线程结束
        if self.pylimo_thread and self.pylimo_thread.is_alive():
            self.pylimo_thread.join(timeout=1.0)
        
        # 释放资源
        if hasattr(self, 'limo') and self.limo is not None:
            try:
                del self.limo
                self.limo = None
            except:
                pass
        
        self.limo_connected = False
        print("已停止pylimo库数据读取")
    
    def start_ros_subscriber(self):
        """启动ROS话题订阅"""
        if self.ros_running:
            return True
        
        try:
            # 初始化ROS节点（如果尚未初始化）
            if not rospy.get_node_uri():
                try:
                    rospy.init_node('limo_status_monitor', anonymous=True)
                except Exception as e:
                    print(f"初始化ROS节点时出错: {str(e)}")
                    # 可能是ROS主节点未运行，启动模拟数据
                    self.start_simulation()
                    return False
            
            # 创建订阅者
            self.ros_subscribers = [
                rospy.Subscriber('/limo/battery_voltage', Float32, self._battery_callback),
                rospy.Subscriber('/limo/motion_mode', Int32, self._motion_mode_callback),
                rospy.Subscriber('/limo/control_mode', Int32, self._control_mode_callback),
                rospy.Subscriber('/limo/velocity', Twist, self._velocity_callback),
                rospy.Subscriber('/limo/steering_angle', Float32, self._steering_angle_callback),
                rospy.Subscriber('/limo/error_code', Int32, self._error_code_callback),
                # 添加对/cmd_vel话题的订阅
                rospy.Subscriber('/cmd_vel', Twist, self._velocity_callback),
                # 添加对/limo_status话题的订阅，用于获取更全面的状态信息
                rospy.Subscriber('/limo_status', Int32, self._limo_status_callback)
            ]
            
            # 停止模拟数据
            self.stop_simulation()
            
            self.ros_running = True
            print("已启动ROS话题订阅")
            return True
        except Exception as e:
            print(f"启动ROS话题订阅时出错: {str(e)}")
            
            # 启动模拟数据生成
            self.start_simulation()
            return False
    
    def stop_ros_subscriber(self):
        """停止ROS话题订阅"""
        if not self.ros_running:
            return
        
        # 取消所有订阅
        for subscriber in self.ros_subscribers:
            try:
                subscriber.unregister()
            except:
                pass
        
        self.ros_subscribers = []
        self.ros_running = False
        print("已停止ROS话题订阅")
    
    def _pylimo_read_loop(self):
        """pylimo库数据读取循环"""
        consecutive_errors = 0
        max_consecutive_errors = 5
        
        while self.pylimo_running:
            try:
                # 使用limomsg模块的全局函数获取状态数据
                try:
                    self.vehicle_state = limomsg.GetVehicleState()
                    self.control_mode = limomsg.GetControlMode()
                    self.battery_voltage = limomsg.GetBatteryVoltage()
                    self.error_code = limomsg.GetErrorCode()
                    self.motion_mode = limomsg.GetMotionMode()
                    self.linear_velocity = limomsg.GetLinearVelocity()
                    self.angular_velocity = limomsg.GetAngularVelocity()
                    self.lateral_velocity = limomsg.GetLateralVelocity()
                    self.steering_angle = limomsg.GetSteeringAngle()
                    
                    # 重置连续错误计数
                    consecutive_errors = 0
                except Exception as data_e:
                    consecutive_errors += 1
                    print(f"获取数据时出现问题 ({consecutive_errors}/{max_consecutive_errors}): {str(data_e)}")
                    
                    # 如果连续错误次数过多，尝试重新初始化
                    if consecutive_errors >= max_consecutive_errors:
                        print("连续错误次数过多，尝试重新初始化LIMO连接...")
                        try:
                            # 释放旧资源
                            if hasattr(self, 'limo') and self.limo is not None:
                                del self.limo
                                self.limo = None
                            
                            # 重新初始化
                            time.sleep(2.0)  # 等待一段时间再重试
                            self.limo = pylimo.LIMO()
                            limomsg._init()
                            print("LIMO连接已重新初始化")
                            consecutive_errors = 0
                        except Exception as reinit_e:
                            print(f"重新初始化LIMO连接失败: {str(reinit_e)}")
                            
                            # 如果重新初始化失败，启动模拟数据
                            if consecutive_errors >= max_consecutive_errors * 2:
                                print("多次重连失败，切换到模拟数据模式")
                                self.pylimo_running = False
                                self.start_simulation()
                                break
                    
                    # 使用上一次的有效数据继续
                    time.sleep(1.0)
                    continue
                
                # 更新历史数据
                with self.data_lock:
                    current_time = time.time()
                    self.timestamps.append(current_time)
                    self.battery_history.append(self.battery_voltage)
                    self.velocity_history.append(self.linear_velocity)
                    
                    # 限制历史数据大小
                    if len(self.timestamps) > self.max_history_size:
                        self.timestamps = self.timestamps[-self.max_history_size:]
                        self.battery_history = self.battery_history[-self.max_history_size:]
                        self.velocity_history = self.velocity_history[-self.max_history_size:]
                
                # 发送信号
                self.status_updated.emit()
                
                # 控制读取频率
                time.sleep(0.1)
            
            except Exception as e:
                print(f"pylimo库数据读取循环中出现问题: {str(e)}")
                time.sleep(1.0)
    
    def _battery_callback(self, msg):
        """电池电压回调函数"""
        with self.data_lock:
            self.battery_voltage = msg.data
            
            # 更新历史数据
            current_time = time.time()
            self.timestamps.append(current_time)
            self.battery_history.append(msg.data)
            
            # 限制历史数据大小
            if len(self.timestamps) > self.max_history_size:
                self.timestamps = self.timestamps[-self.max_history_size:]
                self.battery_history = self.battery_history[-self.max_history_size:]
        
        # 发送信号
        self.status_updated.emit()
    
    def _motion_mode_callback(self, msg):
        """运动模式回调函数"""
        with self.data_lock:
            self.motion_mode = msg.data
            print(f"运动模式更新: {self.motion_mode} ({self.motion_mode_dict.get(self.motion_mode, '未知')})")
        
        # 发送信号
        self.status_updated.emit()
    
    def _control_mode_callback(self, msg):
        """控制模式回调函数"""
        with self.data_lock:
            self.control_mode = msg.data
        
        # 发送信号
        self.status_updated.emit()
    
    def _velocity_callback(self, msg):
        """速度回调函数"""
        with self.data_lock:
            self.linear_velocity = msg.linear.x
            self.angular_velocity = msg.angular.z
            self.lateral_velocity = msg.linear.y
            
            # 更新历史数据
            current_time = time.time()
            self.timestamps.append(current_time)
            self.velocity_history.append(msg.linear.x)
            
            # 限制历史数据大小
            if len(self.timestamps) > self.max_history_size:
                self.timestamps = self.timestamps[-self.max_history_size:]
                self.velocity_history = self.velocity_history[-self.max_history_size:]
        
        # 发送信号
        self.status_updated.emit()
    
    def _steering_angle_callback(self, msg):
        """转向角度回调函数"""
        with self.data_lock:
            self.steering_angle = msg.data
        
        # 发送信号
        self.status_updated.emit()
    
    def _error_code_callback(self, msg):
        """错误代码回调函数"""
        with self.data_lock:
            self.error_code = msg.data
        
        # 发送信号
        self.status_updated.emit()
    
    def _limo_status_callback(self, msg):
        """LIMO状态回调函数，用于获取更全面的状态信息，包括麦轮模式"""
        with self.data_lock:
            # 检查消息中是否包含运动模式信息
            # 通常LIMO状态消息会包含运动模式信息
            # 这里我们假设msg.data的某些位表示运动模式
            # 提取运动模式 (假设运动模式在低4位)
            motion_mode = msg.data & 0x0F
            
            # 如果检测到麦轮模式 (值为2)，则更新运动模式
            if motion_mode == 2:
                self.motion_mode = motion_mode
                print(f"检测到麦轮模式: {self.motion_mode}")
        
        # 发送信号
        self.status_updated.emit()
    
    def _limo_mode_callback(self, msg):
        """limo_mode话题回调函数，用于判断小车的运动模式"""
        with self.data_lock:
            # 直接从limo_mode话题获取运动模式
            self.motion_mode = msg.data
            mode_name = self.motion_mode_dict.get(self.motion_mode, "未知")
            print(f"从/limo_mode话题更新运动模式: {self.motion_mode} ({mode_name})")
        
        # 发送信号
        self.status_updated.emit()
