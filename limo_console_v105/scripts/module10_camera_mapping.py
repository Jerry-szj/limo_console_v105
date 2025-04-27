#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
功能模块10：rtabmap视觉建图
使用大白相机
"""

import os
import sys
import time
import signal
import subprocess

def start_camera_mapping():
    """
    启动rtabmap视觉建图
    使用大白相机
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
        
        # 启动limo_start.launch
        cmd1 = "roslaunch limo_bringup limo_start.launch pub_odom_tf:=true"
        p1 = subprocess.Popen(cmd1, shell=True, env=env, preexec_fn=os.setsid)
        processes.append(p1)
        print(f"已启动: {cmd1}")
        
        # 等待3秒
        time.sleep(3)
        
        # 启动大白相机
        cmd2 = "roslaunch astra_camera dabai_u3.launch"
        p2 = subprocess.Popen(cmd2, shell=True, env=env, preexec_fn=os.setsid)
        processes.append(p2)
        print(f"已启动: {cmd2}")
        
        # 等待3秒
        time.sleep(3)
        
        # 启动rtabmap
        cmd3 = "roslaunch limo_bringup limo_rtabmap_orbbec.launch"
        p3 = subprocess.Popen(cmd3, shell=True, env=env, preexec_fn=os.setsid)
        processes.append(p3)
        print(f"已启动: {cmd3}")
        
        # 等待3秒
        time.sleep(3)
        
        # 启动rviz
        cmd4 = "roslaunch limo_bringup rtabmap_rviz.launch"
        p4 = subprocess.Popen(cmd4, shell=True, env=env, preexec_fn=os.setsid)
        processes.append(p4)
        print(f"已启动: {cmd4}")
        
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
    停止所有相关进程
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
        
    except Exception as e:
        print(f"停止进程时出错: {e}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("用法: python3 module10_camera_mapping.py [start|stop]")
        sys.exit(1)
        
    action = sys.argv[1]
    
    if action == "start":
        start_camera_mapping()
    elif action == "stop":
        stop_all()
    else:
        print(f"未知操作: {action}")
        print("用法: python3 module10_camera_mapping.py [start|stop]")
        sys.exit(1)
