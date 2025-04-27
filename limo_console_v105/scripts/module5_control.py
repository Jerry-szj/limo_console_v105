#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
功能模块5：控制方式
支持键盘控制和鼠标控制两种方式
"""

import os
import sys
import time
import signal
import subprocess

def start_control(control_type):
    """
    启动控制
    
    Args:
        control_type: 控制类型，可选值为"keyboard"(键盘)、"mouse"(鼠标)
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
        
        # 根据控制类型启动相应的控制launch文件
        if control_type == "keyboard":
            cmd = "roslaunch limo_py_drive control.launch control_type:=keyboard"
        elif control_type == "mouse":
            cmd = "roslaunch limo_py_drive control.launch control_type:=mouse"
        else:
            raise ValueError(f"未知的控制类型: {control_type}")
        
        p = subprocess.Popen(cmd, shell=True, env=env, preexec_fn=os.setsid)
        processes.append(p)
        print(f"已启动: {cmd}")
        
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
        print("用法: python3 module5_control.py [start|stop] [type]")
        print("type可选值: keyboard(键盘), mouse(鼠标)")
        sys.exit(1)
        
    action = sys.argv[1]
    
    if action == "start":
        if len(sys.argv) < 3:
            print("启动控制时必须指定控制类型")
            print("用法: python3 module5_control.py start [type]")
            print("type可选值: keyboard(键盘), mouse(鼠标)")
            sys.exit(1)
        
        control_type = sys.argv[2]
        start_control(control_type)
    elif action == "stop":
        stop_all()
    else:
        print(f"未知操作: {action}")
        print("用法: python3 module5_control.py [start|stop] [type]")
        print("type可选值: keyboard(键盘), mouse(鼠标)")
        sys.exit(1)
