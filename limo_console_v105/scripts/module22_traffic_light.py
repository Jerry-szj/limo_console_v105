#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
功能模块22：红绿灯识别
使用大白相机进行红绿灯识别
"""

import os
import sys
import time
import signal
import subprocess
import threading

def start_traffic_light_detection():
    """
    启动红绿灯识别功能
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
        # cmd1 = "roslaunch astra_camera dabai_u3.launch"
        # p1 = subprocess.Popen(cmd1, shell=True, env=env, preexec_fn=os.setsid)
        # processes.append(p1)
        # print(f"已启动: {cmd1}")
        
        # # 等待3秒
        # time.sleep(3)
        
        # 启动红绿灯识别
        cmd2 = "roslaunch limo_deeplearning traffic_detect.launch"
        p2 = subprocess.Popen(cmd2, shell=True, env=env, preexec_fn=os.setsid)
        processes.append(p2)
        print(f"已启动: {cmd2}")
        
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

def stop_traffic_light_detection():
    """
    停止红绿灯识别功能
    """
    try:
        # 查找相关进程
        cmd = "ps aux | grep -E 'traffic_detect.launch|dabai_u3.launch' | grep -v grep"
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
        
        # 等待3秒
        time.sleep(3)
        
        # 关闭所有相关进程
        subprocess.run("pkill -f 'roslaunch limo_deeplearning traffic_detect.launch'", shell=True)
        subprocess.run("pkill -f 'roslaunch astra_camera dabai_u3.launch'", shell=True)
        
        print("红绿灯识别已停止")
        
    except Exception as e:
        print(f"停止红绿灯识别时出错: {e}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("用法: python3 module22_traffic_light.py [start|stop]")
        sys.exit(1)
        
    action = sys.argv[1]
    
    if action == "start":
        start_traffic_light_detection()
    elif action == "stop":
        stop_traffic_light_detection()
    else:
        print(f"未知操作: {action}")
        print("用法: python3 module22_traffic_light.py [start|stop]")
        sys.exit(1)
