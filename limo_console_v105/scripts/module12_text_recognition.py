#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
功能模块12：识别文字
启动文字识别功能
"""

import os
import sys
import time
import signal
import subprocess

def start_text_recognition():
    """
    启动文字识别功能
    1. 启动roscore
    2. 等待3秒
    3. 启动detect_node.py
    4. 等待3秒
    5. 启动echo监听结果
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
        
        # 启动roscore
        cmd1 = "roscore"
        p1 = subprocess.Popen(cmd1, shell=True, env=env, preexec_fn=os.setsid)
        processes.append(p1)
        print(f"已启动: {cmd1}")
        
        # 等待3秒
        time.sleep(3)
        
        # 启动detect_node.py
        cmd2 = "rosrun vision detect_node.py"
        p2 = subprocess.Popen(cmd2, shell=True, env=env, preexec_fn=os.setsid)
        processes.append(p2)
        print(f"已启动: {cmd2}")
        
        # 等待3秒
        time.sleep(3)
        
        # 启动echo监听结果
        cmd3 = "rostopic echo /detect_word_reslut"
        p3 = subprocess.Popen(cmd3, shell=True, env=env, preexec_fn=os.setsid)
        processes.append(p3)
        print(f"已启动: {cmd3}")
        
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
        print("正在停止所有文字识别相关进程...")
        
        # 首先尝试优雅地关闭detect_node.py进程
        try:
            # 查找detect_node.py相关进程
            cmd = "ps aux | grep -E 'detect_node.py' | grep -v grep"
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
                    print(f"正在优雅地终止detect_node.py进程 {pid}...")
                    os.kill(int(pid), signal.SIGINT)
                except Exception as e:
                    print(f"发送SIGINT到进程 {pid} 时出错: {str(e)}")
            
            # 等待2秒让进程有时间优雅地退出
            time.sleep(2)
            
            # 检查是否还有detect_node.py进程在运行
            cmd = "ps aux | grep -E 'detect_node.py' | grep -v grep"
            output = subprocess.check_output(cmd, shell=True).decode('utf-8')
            
            # 如果还有进程，强制终止
            if output.strip():
                print("一些detect_node.py进程仍在运行，正在强制终止...")
                for line in output.strip().split('\n'):
                    if not line:
                        continue
                    
                    parts = line.split()
                    if len(parts) < 2:
                        continue
                        
                    pid = parts[1]
                    try:
                        # 发送SIGKILL信号
                        print(f"正在强制终止进程 {pid}...")
                        os.kill(int(pid), signal.SIGKILL)
                    except Exception as e:
                        print(f"发送SIGKILL到进程 {pid} 时出错: {str(e)}")
        except Exception as e:
            print(f"终止detect_node.py进程时出错: {str(e)}")
        
        # 然后终止所有其他ROS相关进程
        try:
            # 查找所有ROS相关进程
            cmd = "ps aux | grep -E 'roslaunch|rosrun|roscore|rostopic|vision' | grep -v grep"
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
                    print(f"正在终止ROS进程 {pid}...")
                    os.kill(int(pid), signal.SIGINT)
                except Exception as e:
                    print(f"发送SIGINT到进程 {pid} 时出错: {str(e)}")
        except Exception as e:
            print(f"终止ROS进程时出错: {str(e)}")
        
        # 等待3秒
        print("等待进程终止...")
        time.sleep(3)
        
        # 检查是否还有ROS相关进程在运行
        try:
            cmd = "ps aux | grep -E 'roslaunch|rosrun|roscore|rostopic|vision|detect_node.py' | grep -v grep"
            output = subprocess.check_output(cmd, shell=True).decode('utf-8')
            
            # 如果还有进程，强制终止
            if output.strip():
                print("一些ROS进程仍在运行，正在强制终止...")
                for line in output.strip().split('\n'):
                    if not line:
                        continue
                    
                    parts = line.split()
                    if len(parts) < 2:
                        continue
                        
                    pid = parts[1]
                    try:
                        # 发送SIGKILL信号
                        print(f"正在强制终止进程 {pid}...")
                        os.kill(int(pid), signal.SIGKILL)
                    except Exception as e:
                        print(f"发送SIGKILL到进程 {pid} 时出错: {str(e)}")
        except Exception as e:
            print(f"强制终止ROS进程时出错: {str(e)}")
        
        # 关闭所有终端窗口
        try:
            print("正在关闭所有终端窗口...")
            subprocess.run("pkill -f xterm", shell=True)
        except Exception as e:
            print(f"关闭终端窗口时出错: {str(e)}")
        
        print("所有文字识别相关进程已停止")
        
    except Exception as e:
        print(f"停止进程时出错: {str(e)}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("用法: python3 module12_text_recognition.py [start|stop]")
        sys.exit(1)
        
    action = sys.argv[1]
    
    if action == "start":
        start_text_recognition()
    elif action == "stop":
        stop_all()
    else:
        print(f"未知操作: {action}")
        print("用法: python3 module12_text_recognition.py [start|stop]")
        sys.exit(1)
