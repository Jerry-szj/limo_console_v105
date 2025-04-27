#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
module1_gmapping.py - Gmapping建图模块
"""

import sys
import time
import signal
import subprocess
import threading
import os

def signal_handler(sig, frame):
    """信号处理函数，用于处理Ctrl+C"""
    print("接收到终止信号，正在停止...")
    sys.exit(0)

def save_map():
    """保存地图功能"""
    print("开始保存地图...")
    
    try:
        # 切换到地图保存目录
        map_dir = os.path.expanduser("~/agilex_ws/src/limo_ros/limo_bringup/maps/")
        if not os.path.exists(map_dir):
            os.makedirs(map_dir)
            print(f"创建地图目录: {map_dir}")
        
        print(f"切换到地图目录: {map_dir}")
        
        # 等待3秒
        print("等待3秒...")
        time.sleep(3)
        
        # 保存地图
        save_process = subprocess.Popen(
            ["rosrun", "map_server", "map_saver", "-f", "map1"],
            cwd=map_dir,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            universal_newlines=True
        )
        
        # 读取输出
        while True:
            output = save_process.stdout.readline()
            if output:
                print(output.strip())
            
            # 检查进程是否结束
            if save_process.poll() is not None:
                break
            
            time.sleep(0.1)
        
        # 检查是否成功
        if save_process.returncode == 0:
            print(f"地图已成功保存到: {os.path.join(map_dir, 'map1.pgm')}")
        else:
            error = save_process.stderr.read()
            print(f"保存地图失败: {error}")
    
    except Exception as e:
        print(f"保存地图时出错: {str(e)}")

def start_module(*params):
    """启动Gmapping建图模块
    
    参数:
        params: 附加参数
    """
    print(f"启动 Gmapping建图，参数: {params}")
    
    try:
        # 检查基础节点是否已启动
        print("检查基础节点是否已启动...")
        base_node_running = False
        try:
            # 检查limo_base进程是否在运行
            cmd = "ps aux | grep -E 'limo_base|limo_start.launch' | grep -v grep"
            output = subprocess.check_output(cmd, shell=True).decode('utf-8')
            if output.strip():
                base_node_running = True
                print("检测到基础节点已启动")
            else:
                print("未检测到基础节点，将先启动基础节点...")
        except Exception as e:
            print(f"检查基础节点时出错: {str(e)}")
        
        # 如果基础节点未运行，先启动基础节点
        if not base_node_running:
            print("正在启动基础节点...")
            base_process = subprocess.Popen(
                ["roslaunch", "limo_bringup", "limo_start.launch", "pub_odom_tf:=false"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                universal_newlines=True
            )
            
            # 等待5秒，确保基础节点完全启动
            print("等待5秒，确保基础节点完全启动...")
            time.sleep(5)
        
        # 启动Gmapping建图节点
        print("正在启动Gmapping建图节点...")
        process = subprocess.Popen(
            ["roslaunch", "limo_bringup", "limo_gmapping.launch"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            universal_newlines=True
        )
        
        # 读取输出
        while True:
            output = process.stdout.readline()
            if output:
                print(output.strip())
            
            # 检查进程是否结束
            if process.poll() is not None:
                break
            
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        print("用户中断，停止 Gmapping建图")
    except Exception as e:
        print(f"运行 Gmapping建图 时出错: {str(e)}")

if __name__ == "__main__":
    # 注册信号处理函数
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # 解析命令行参数
    if len(sys.argv) < 2:
        print("用法: python module1_gmapping.py [start|stop|save] [参数...]")
        sys.exit(1)
    
    action = sys.argv[1]
    params = sys.argv[2:] if len(sys.argv) > 2 else []
    
    if action == "start":
        start_module(*params)
    elif action == "stop":
        # 停止操作由脚本执行器通过发送信号实现
        print("停止 Gmapping建图")
    elif action == "save":
        # 保存地图
        save_map()
    else:
        print(f"无效的动作: {action}")
        sys.exit(1)
