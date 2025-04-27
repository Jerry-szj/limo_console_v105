#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
功能模块9：Dabai DC1
使用大白相机
在启动相机5秒后运行rviz
结束时清除所有任务包括roscore和rosmaster
"""

import os
import sys
import time
import signal
import subprocess
import threading

def run_rviz_after_delay():
    """
    延迟5秒后运行rviz
    """
    time.sleep(5)
    try:
        print("正在启动rviz...")
        env = os.environ.copy()
        env["ROS_MASTER_URI"] = "http://master:11311"
        subprocess.Popen("rosrun rviz rviz", shell=True, env=env, preexec_fn=os.setsid)
        print("rviz已启动")
    except Exception as e:
        print(f"启动rviz时出错: {e}")

def start_camera():
    """
    启动大白相机
    """
    processes = []
    
    try:
        # 切换到工作目录
        os.chdir(os.path.expanduser("~/agilex_ws"))
        
        # 设置环境变量
        env = os.environ.copy()
        env["ROS_MASTER_URI"] = "http://master:11311"
        
        # 运行source命令
        subprocess.run("source devel/setup.bash", shell=True, executable="/bin/bash")
        
        # 启动大白相机
        cmd = "roslaunch astra_camera dabai_u3.launch"
        p = subprocess.Popen(cmd, shell=True, env=env, preexec_fn=os.setsid)
        processes.append(p)
        print(f"已启动: {cmd}")
        
        # 创建线程，延迟5秒后启动rviz
        rviz_thread = threading.Thread(target=run_rviz_after_delay)
        rviz_thread.daemon = True
        rviz_thread.start()
        
        # 等待进程结束或被终止
        while all(p.poll() is None for p in processes):
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("接收到终止信号，正在停止所有进程...")
    except Exception as e:
        print(f"发生错误: {e}")
    finally:
        # 终止所有进程
        for p in processes:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGINT)
                time.sleep(1)
                if p.poll() is None:
                    os.killpg(os.getpgid(p.pid), signal.SIGTERM)
            except:
                pass
        
        print("所有进程已停止")

def stop_all():
    """
    停止所有相关进程，包括roscore和rosmaster
    """
    try:
        # 查找相关进程
        cmd = "ps aux | grep -E 'roslaunch|rosrun' | grep -v grep"
        output = subprocess.check_output(cmd, shell=True).decode('utf-8')
        
        # 提取PID
        for line in output.strip().split('\n'):
            if not line:
                continue
            
            parts = line.split()
            if len(parts) < 2:
                continue
                
            pid = parts[1]
            try:
                # 发送SIGINT信号 (Ctrl+C)
                os.kill(int(pid), signal.SIGINT)
                print(f"已发送终止信号到进程 {pid}")
            except:
                pass
        
        # 等待5秒
        time.sleep(5)
        
        # 关闭所有终端窗口
        subprocess.run("pkill -f xterm", shell=True)
        
        # 清除roscore和rosmaster
        print("正在清除roscore和rosmaster...")
        subprocess.run("killall -9 roscore rosmaster", shell=True)
        subprocess.run("killall -9 rosout roslaunch", shell=True)
        print("已清除roscore和rosmaster")
        
    except Exception as e:
        print(f"停止进程时出错: {e}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("用法: python3 module9_camera.py [start|stop]")
        sys.exit(1)
        
    action = sys.argv[1]
    
    if action == "start":
        start_camera()
    elif action == "stop":
        stop_all()
    else:
        print(f"未知操作: {action}")
        print("用法: python3 module9_camera.py [start|stop]")
        sys.exit(1)
