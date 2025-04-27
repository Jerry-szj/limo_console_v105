#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import time
import signal
import subprocess

def signal_handler(sig, frame):
    """信号处理函数，用于处理Ctrl+C"""
    print("接收到终止信号，正在停止...")
    stop_cartographer()
    sys.exit(0)

def start_cartographer():
    """启动Cartographer建图"""
    print("正在启动Cartographer建图...")
    
    # 确保ROS环境已设置
    ros_setup = "source /opt/ros/noetic/setup.bash && source ~/agilex_ws/devel/setup.bash && cd ~/agilex_ws/ && source install_isolated/local_setup.bash"
    
    # 启动激光雷达和底盘
    cmd_lidar = f"{ros_setup} && roslaunch limo_bringup limo_start.launch"
    p_lidar = subprocess.Popen(cmd_lidar, shell=True, executable='/bin/bash')
    
    # 等待激光雷达启动
    print("等待激光雷达启动...")
    time.sleep(8)
    
    # 启动Cartographer节点
    cmd_cart = f"{ros_setup} && roslaunch limo_bringup limo_cartographer.launch"
    p_cart = subprocess.Popen(cmd_cart, shell=True, executable='/bin/bash')
    
    # 等待Cartographer节点启动
    print("等待Cartographer节点启动...")
    time.sleep(5)
    
    # 启动RViz可视化
    # rviz_config = os.path.join(os.path.expanduser("~"), "agilex_ws/src/limo_ros/limo_bringup/rviz/cartographer.rviz")
    # cmd_rviz = f"{ros_setup} && rosrun rviz rviz -d {rviz_config}"
    # p_rviz = subprocess.Popen(cmd_rviz, shell=True, executable='/bin/bash')
    
    print("Cartographer建图已启动")
    
    # 等待用户中断
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        stop_cartographer()

def stop_cartographer():
    """停止Cartographer建图"""
    print("正在停止Cartographer建图...")
    
    # 终止所有相关进程
    try:
        # 终止RViz
        subprocess.run("pkill -f rviz", shell=True)
        
        # 终止Cartographer节点
        subprocess.run("rosnode kill /cartographer_node", shell=True, stderr=subprocess.DEVNULL)
        subprocess.run("rosnode kill /cartographer_occupancy_grid_node", shell=True, stderr=subprocess.DEVNULL)
        
        # 终止激光雷达节点
        subprocess.run("rosnode kill /ydlidar_node", shell=True, stderr=subprocess.DEVNULL)
        
        # 终止其他相关节点
        subprocess.run("pkill -f cartographer", shell=True, stderr=subprocess.DEVNULL)
        subprocess.run("pkill -f limo_start", shell=True, stderr=subprocess.DEVNULL)
        
        print("Cartographer建图已停止")
    except Exception as e:
        print(f"停止Cartographer建图时出错: {str(e)}")

def save_cartographer_map():
    """保存Cartographer地图"""
    print("正在保存Cartographer地图...")
    
    try:
        # 创建地图保存目录
        map_dir = os.path.join(os.path.expanduser("~"), "agilex_ws/src/limo_ros/limo_bringup/maps")
        os.makedirs(map_dir, exist_ok=True)
        
        # 设置地图文件路径
        map_path = os.path.join(map_dir, "cartographer_map")
        
        # 保存地图
        ros_setup = "source /opt/ros/noetic/setup.bash && source ~/agilex_ws/devel/setup.bash"
        cmd = f"{ros_setup} && rosrun cartographer_ros cartographer_pbstream_to_ros_map -map_filestem={map_path} -pbstream_filename=/tmp/cartographer_map.pbstream -resolution=0.05"
        
        # 执行保存命令
        print(f"执行命令: {cmd}")
        result = subprocess.run(cmd, shell=True, executable='/bin/bash', stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        
        if result.returncode == 0:
            print(f"Cartographer地图已保存到: {map_path}")
            return True, f"Cartographer地图已保存到: {map_path}"
        else:
            error_msg = f"保存Cartographer地图失败: {result.stderr}"
            print(error_msg)
            return False, error_msg
    
    except Exception as e:
        error_msg = f"保存Cartographer地图时出错: {str(e)}"
        print(error_msg)
        return False, error_msg

def detect_mapping_type():
    """检测当前运行的建图类型"""
    try:
        # 设置ROS环境
        ros_setup = "source /opt/ros/noetic/setup.bash && source ~/agilex_ws/devel/setup.bash"
        
        # 检查Cartographer节点是否运行
        cmd = f"{ros_setup} && rosnode list | grep cartographer_node"
        result = subprocess.run(cmd, shell=True, executable='/bin/bash', stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        
        if "/cartographer_node" in result.stdout:
            return "cartographer"
        
        # 检查Gmapping节点是否运行
        cmd = f"{ros_setup} && rosnode list | grep slam_gmapping"
        result = subprocess.run(cmd, shell=True, executable='/bin/bash', stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        
        if "/slam_gmapping" in result.stdout:
            return "gmapping"
        
        return None
    
    except Exception as e:
        print(f"检测建图类型时出错: {str(e)}")
        return None

if __name__ == "__main__":
    # 注册信号处理函数
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # 解析命令行参数
    if len(sys.argv) < 2:
        print("用法: python module2_cartographer.py [start|stop|save]")
        sys.exit(1)
    
    action = sys.argv[1]
    
    if action == "start":
        start_cartographer()
    elif action == "stop":
        stop_cartographer()
    elif action == "save":
        # 检测建图类型
        mapping_type = detect_mapping_type()
        
        if mapping_type == "cartographer":
            success, message = save_cartographer_map()
            if not success:
                sys.exit(1)
        else:
            print("未检测到正在运行的Cartographer建图，请先启动Cartographer")
            sys.exit(1)
    else:
        print(f"无效的动作: {action}")
        print("用法: python module2_cartographer.py [start|stop|save]")
        sys.exit(1)
