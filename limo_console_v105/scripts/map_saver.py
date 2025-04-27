#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
map_saver.py - 一键保存地图功能模块
支持Gmapping和Cartographer两种建图方式
"""

import os
import sys
import time
import signal
import subprocess
import threading
import datetime

def signal_handler(sig, frame):
    """信号处理函数，用于处理Ctrl+C"""
    print("接收到终止信号，正在停止...")
    sys.exit(0)

def save_gmapping_map():
    """保存Gmapping地图"""
    print("开始保存Gmapping地图...")
    
    try:
        # 生成带时间戳的地图名称
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        map_name = f"map11"
        
        # 切换到地图保存目录
        map_dir = os.path.expanduser("~/agilex_ws/src/limo_ros/limo_bringup/maps/")
        if not os.path.exists(map_dir):
            os.makedirs(map_dir)
            print(f"创建地图目录: {map_dir}")
        
        print(f"切换到地图目录: {map_dir}")
        
        # 保存地图
        save_process = subprocess.Popen(
            ["rosrun", "map_server", "map_saver", "-f", map_name],
            cwd=map_dir,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            universal_newlines=True
        )
        
        # 读取输出
        stdout, stderr = save_process.communicate()
        
        # 检查是否成功
        if save_process.returncode == 0:
            print(f"Gmapping地图已成功保存到: {os.path.join(map_dir, map_name)}.pgm")
            return True, f"Gmapping地图已成功保存到: {os.path.join(map_dir, map_name)}.pgm"
        else:
            print(f"保存Gmapping地图失败: {stderr}")
            return False, f"保存Gmapping地图失败: {stderr}"
    
    except Exception as e:
        print(f"保存Gmapping地图时出错: {str(e)}")
        return False, f"保存Gmapping地图时出错: {str(e)}"

def save_cartographer_map():
    """保存Cartographer地图"""
    print("开始保存Cartographer地图...")
    
    try:
        # 生成带时间戳的地图名称
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        map_name = f"cartographer_map_{timestamp}"
        map_path = os.path.expanduser(f"~/agilex_ws/src/limo_ros/limo_bringup/maps/{map_name}")
        
        # 确保地图目录存在
        map_dir = os.path.expanduser("~/agilex_ws/src/limo_ros/limo_bringup/maps/")
        if not os.path.exists(map_dir):
            os.makedirs(map_dir)
            print(f"创建地图目录: {map_dir}")
        
        # 结束轨迹
        print("步骤1: 结束轨迹...")
        cmd1 = ["rosservice", "call", "/finish_trajectory", "0"]
        p1 = subprocess.run(cmd1, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        if p1.returncode != 0:
            print(f"结束轨迹失败: {p1.stderr}")
            return False, f"结束轨迹失败: {p1.stderr}"
        
        # 等待4秒
        print("等待4秒...")
        time.sleep(4)
        
        # 写入状态
        print("步骤2: 写入状态...")
        cmd2 = ["rosservice", "call", "/write_state", f"{{filename: '{map_path}.pbstream'}}"]
        p2 = subprocess.run(cmd2, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        if p2.returncode != 0:
            print(f"写入状态失败: {p2.stderr}")
            return False, f"写入状态失败: {p2.stderr}"
        
        # 等待4秒
        print("等待4秒...")
        time.sleep(4)
        
        # 转换地图
        print("步骤3: 转换地图...")
        cmd3 = [
            "rosrun", "cartographer_ros", "cartographer_pbstream_to_ros_map",
            f"-map_filestem={map_path}",
            f"-pbstream_filename={map_path}.pbstream",
            "-resolution=0.05"
        ]
        p3 = subprocess.run(cmd3, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        if p3.returncode != 0:
            print(f"转换地图失败: {p3.stderr}")
            return False, f"转换地图失败: {p3.stderr}"
        
        print(f"Cartographer地图已成功保存到: {map_path}.pgm")
        return True, f"Cartographer地图已成功保存到: {map_path}.pgm"
            
    except Exception as e:
        print(f"保存Cartographer地图时出错: {str(e)}")
        return False, f"保存Cartographer地图时出错: {str(e)}"

def detect_mapping_type():
    """检测当前运行的建图类型"""
    try:
        # 检查运行中的节点
        result = subprocess.run(
            ["rosnode", "list"], 
            stdout=subprocess.PIPE, 
            stderr=subprocess.PIPE, 
            text=True
        )
        
        if result.returncode != 0:
            print("无法获取ROS节点列表，请确保ROS环境已启动")
            return None
        
        nodes = result.stdout.strip().split('\n')
        
        # 检查是否有Gmapping节点
        if any("slam_gmapping" in node for node in nodes):
            return "gmapping"
        
        # 检查是否有Cartographer节点
        if any("cartographer" in node for node in nodes):
            return "cartographer"
        
        # 检查运行中的话题
        result = subprocess.run(
            ["rostopic", "list"], 
            stdout=subprocess.PIPE, 
            stderr=subprocess.PIPE, 
            text=True
        )
        
        topics = result.stdout.strip().split('\n')
        
        # 通过话题判断
        if any("/map" in topic for topic in topics):
            # 进一步区分
            if any("/finish_trajectory" in topic for topic in topics):
                return "cartographer"
            else:
                return "gmapping"
        
        return None
    
    except Exception as e:
        print(f"检测建图类型时出错: {str(e)}")
        return None

def save_map():
    """一键保存地图，自动检测建图类型"""
    # 检测建图类型
    mapping_type = detect_mapping_type()
    
    if mapping_type is None:
        print("未检测到正在运行的建图进程，请先启动建图")
        return False, "未检测到正在运行的建图进程，请先启动建图"
    
    print(f"检测到正在运行的建图类型: {mapping_type}")
    
    # 根据建图类型调用相应的保存函数
    if mapping_type == "gmapping":
        return save_gmapping_map()
    elif mapping_type == "cartographer":
        return save_cartographer_map()
    else:
        print(f"不支持的建图类型: {mapping_type}")
        return False, f"不支持的建图类型: {mapping_type}"

if __name__ == "__main__":
    # 注册信号处理函数
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # 解析命令行参数
    if len(sys.argv) < 2:
        print("用法: python map_saver.py [save|gmapping|cartographer]")
        sys.exit(1)
    
    action = sys.argv[1]
    
    if action == "save":
        # 自动检测并保存
        success, message = save_map()
        if not success:
            sys.exit(1)
    elif action == "gmapping":
        # 强制使用Gmapping保存
        success, message = save_gmapping_map()
        if not success:
            sys.exit(1)
    elif action == "cartographer":
        # 强制使用Cartographer保存
        success, message = save_cartographer_map()
        if not success:
            sys.exit(1)
    else:
        print(f"无效的动作: {action}")
        print("用法: python map_saver.py [save|gmapping|cartographer]")
        sys.exit(1)
