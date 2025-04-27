#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
脚本执行模块
负责执行Python模块脚本，替代原来的sh脚本
"""

import os
import sys
import signal
import subprocess
import threading
from PyQt5.QtCore import QObject, pyqtSignal

class ScriptExecutor(QObject):
    """脚本执行器类，负责执行Python模块脚本"""
    
    # 定义信号
    output_received = pyqtSignal(str)
    error_received = pyqtSignal(str)
    
    def __init__(self, parent=None):
        super(ScriptExecutor, self).__init__(parent)
        
        # 进程字典，用于存储正在运行的进程
        self.processes = {}
        
        # 脚本目录
        self.script_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "scripts")
        
        # 模块ID到脚本文件的映射
        self.module_scripts = {
            1: "module1_gmapping.py",
            2: "module2_cartographer.py",
            3: "module3_check_lidar.py",
            4: "module4_navigation.py",
            5: "module5_control.py",
            6: "module6_integrated_control.py",
            7: "module7_base_node.py",
            8: "module8_path_patrol.py",
            9: "module9_camera.py",
            10: "module10_camera_mapping.py",
            11: "module11_camera_navigation.py",
            12: "module12_text_recognition.py",
            14: "module14_voice_to_text.py",
            15: "module15_voice_control.py",
            16: "module16_rviz.py",
            18: "module18_color_recognition.py",
            19: "module19_rrt_mapping.py",
            20: "module20_lidar_following.py",
            21: "module21_lane_following.py",
            22: "module22_traffic_light.py",
            23: "module23_yolo.py"
        }
    
    def execute_script(self, module_id, action, *params):
        """执行脚本
        
        参数:
            module_id: 模块ID
            action: 动作，"start"或"stop"
            params: 附加参数
        """
        # 检查模块ID是否有效
        if module_id not in self.module_scripts:
            self.error_received.emit(f"无效的模块ID: {module_id}")
            return False
        
        # 获取脚本文件名
        script_file = self.module_scripts[module_id]
        script_path = os.path.join(self.script_dir, script_file)
        
        # 检查脚本文件是否存在
        if not os.path.exists(script_path):
            # 如果脚本文件不存在，尝试创建一个基本的脚本文件
            self._create_basic_script(module_id, script_path)
        
        # 根据动作执行不同的操作
        if action == "start":
            return self._start_script(module_id, script_path, *params)
        elif action == "stop":
            return self._stop_script(module_id)
        else:
            self.error_received.emit(f"无效的动作: {action}")
            return False
    
    def _start_script(self, module_id, script_path, *params):
        """启动脚本"""
        # 如果该模块已经有进程在运行，先停止它
        if module_id in self.processes and self.processes[module_id]["process"].poll() is None:
            self._stop_script(module_id)
        
        try:
            # 构建命令
            cmd = [sys.executable, script_path, "start"]
            cmd.extend(params)
            
            # 启动进程
            process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                universal_newlines=True,
                bufsize=1
            )
            
            # 存储进程信息
            self.processes[module_id] = {
                "process": process,
                "stdout_thread": None,
                "stderr_thread": None
            }
            
            # 启动输出读取线程
            stdout_thread = threading.Thread(
                target=self._read_output,
                args=(process.stdout, self.output_received)
            )
            stdout_thread.daemon = True
            stdout_thread.start()
            
            stderr_thread = threading.Thread(
                target=self._read_output,
                args=(process.stderr, self.error_received)
            )
            stderr_thread.daemon = True
            stderr_thread.start()
            
            # 存储线程
            self.processes[module_id]["stdout_thread"] = stdout_thread
            self.processes[module_id]["stderr_thread"] = stderr_thread
            
            self.output_received.emit(f"已启动模块 {module_id}")
            return True
        
        except Exception as e:
            self.error_received.emit(f"启动脚本时出错: {str(e)}")
            return False
    
    def _stop_script(self, module_id):
        """停止脚本"""
        if module_id not in self.processes:
            self.error_received.emit(f"模块 {module_id} 未在运行")
            return False
        
        process_info = self.processes[module_id]
        process = process_info["process"]
        
        # 检查进程是否仍在运行
        if process.poll() is None:
            try:
                # 发送SIGTERM信号
                process.terminate()
                
                # 等待进程结束（最多等待3秒）
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    # 如果超时，发送SIGKILL信号
                    process.kill()
                
                self.output_received.emit(f"已停止模块 {module_id}")
                return True
            
            except Exception as e:
                self.error_received.emit(f"停止脚本时出错: {str(e)}")
                return False
        else:
            # 进程已经结束
            self.output_received.emit(f"模块 {module_id} 已经停止")
            return True
    
    def terminate_all(self):
        """终止所有进程"""
        for module_id in list(self.processes.keys()):
            self._stop_script(module_id)
    
    def _read_output(self, pipe, signal):
        """读取输出管道并发送信号"""
        for line in pipe:
            signal.emit(line.strip())
    
    def _create_basic_script(self, module_id, script_path):
        """创建基本的脚本文件"""
        try:
            # 获取模块名称
            module_name = os.path.basename(script_path).replace(".py", "")
            
            # 创建基本脚本内容
            script_content = f"""#!/usr/bin/env python3
# -*- coding: utf-8 -*-
\"\"\"
{module_name} - LIMO功能模块脚本
\"\"\"

import sys
import time
import signal
import subprocess
import threading

def signal_handler(sig, frame):
    \"\"\"信号处理函数，用于处理Ctrl+C\"\"\"
    print("接收到终止信号，正在停止...")
    sys.exit(0)

def start_module(*params):
    \"\"\"启动模块
    
    参数:
        params: 附加参数
    \"\"\"
    print(f"启动 {module_name}，参数: {{params}}")
    
    # TODO: 在这里实现模块的启动逻辑
    # 例如，启动ROS节点、运行特定命令等
    
    try:
        # 示例：运行一个简单的命令
        # subprocess.run(["roslaunch", "limo_bringup", "limo_start.launch"])
        
        # 为了演示，这里只是简单地等待
        while True:
            print(f"{module_name} 正在运行...")
            time.sleep(5)
    
    except KeyboardInterrupt:
        print(f"用户中断，停止 {module_name}")
    except Exception as e:
        print(f"运行 {module_name} 时出错: {{str(e)}}")

if __name__ == "__main__":
    # 注册信号处理函数
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # 解析命令行参数
    if len(sys.argv) < 2:
        print("用法: python {module_name}.py [start|stop] [参数...]")
        sys.exit(1)
    
    action = sys.argv[1]
    params = sys.argv[2:] if len(sys.argv) > 2 else []
    
    if action == "start":
        start_module(*params)
    elif action == "stop":
        # 停止操作由脚本执行器通过发送信号实现
        print(f"停止 {module_name}")
    else:
        print(f"无效的动作: {{action}}")
        sys.exit(1)
"""
            
            # 创建目录（如果不存在）
            os.makedirs(os.path.dirname(script_path), exist_ok=True)
            
            # 写入脚本文件
            with open(script_path, "w") as f:
                f.write(script_content)
            
            # 设置执行权限
            os.chmod(script_path, 0o755)
            
            self.output_received.emit(f"已创建基本脚本文件: {script_path}")
            return True
        
        except Exception as e:
            self.error_received.emit(f"创建基本脚本文件时出错: {str(e)}")
            return False
