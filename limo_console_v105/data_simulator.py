import threading
import time
import random
import numpy as np

class DataSimulator:
    """数据模拟器类，用于在无法获取真实数据时生成模拟数据"""
    
    def __init__(self, status_monitor):
        self.status_monitor = status_monitor
        self.running = False
        self.thread = None
        
        # 模拟数据参数
        self.battery_voltage_base = 24.0  # 基础电池电压
        self.battery_voltage_var = 0.5    # 电池电压变化范围
        self.linear_velocity_base = 0.0   # 基础线速度
        self.linear_velocity_var = 0.2    # 线速度变化范围
        self.angular_velocity_base = 0.0  # 基础角速度
        self.angular_velocity_var = 0.1   # 角速度变化范围
        
    def start(self):
        """启动数据模拟"""
        if self.running:
            return
            
        self.running = True
        self.thread = threading.Thread(target=self._simulation_loop)
        self.thread.daemon = True
        self.thread.start()
        print("已启动数据模拟")
        
    def stop(self):
        """停止数据模拟"""
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)
        print("已停止数据模拟")
        
    def _simulation_loop(self):
        """模拟数据生成循环"""
        while self.running:
            try:
                # 生成模拟数据
                with self.status_monitor.data_lock:
                    # 模拟电池电压（缓慢下降趋势）
                    self.status_monitor.battery_voltage = self.battery_voltage_base + random.uniform(-self.battery_voltage_var/2, self.battery_voltage_var/2)
                    self.battery_voltage_base = max(20.0, self.battery_voltage_base - 0.001)  # 缓慢下降
                    
                    # 模拟速度（随机波动）
                    self.status_monitor.linear_velocity = self.linear_velocity_base + random.uniform(-self.linear_velocity_var, self.linear_velocity_var)
                    self.status_monitor.angular_velocity = self.angular_velocity_base + random.uniform(-self.angular_velocity_var, self.angular_velocity_var)
                    
                    # 更新历史数据
                    current_time = time.time()
                    self.status_monitor.timestamps.append(current_time)
                    self.status_monitor.battery_history.append(self.status_monitor.battery_voltage)
                    self.status_monitor.velocity_history.append(self.status_monitor.linear_velocity)
                    
                    # 限制历史数据大小
                    if len(self.status_monitor.timestamps) > self.status_monitor.max_history_size:
                        self.status_monitor.timestamps = self.status_monitor.timestamps[-self.status_monitor.max_history_size:]
                        self.status_monitor.battery_history = self.status_monitor.battery_history[-self.status_monitor.max_history_size:]
                        self.status_monitor.velocity_history = self.status_monitor.velocity_history[-self.status_monitor.max_history_size:]
                
                # 发送信号
                self.status_monitor.status_updated.emit()
                
                # 控制更新频率
                time.sleep(0.1)
                
            except Exception as e:
                print(f"数据模拟过程中出错: {str(e)}")
                time.sleep(1.0)
