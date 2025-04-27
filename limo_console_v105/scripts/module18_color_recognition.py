#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
功能模块18：颜色识别
基于摄像头的颜色分类，识别红绿蓝黄橙这几个颜色，并用方框框起识别到的区域
支持颜色跟踪功能
"""

import os
import sys
import time
import signal
import subprocess
import threading
import cv2
import numpy as np

# 定义颜色范围（HSV空间）
COLOR_RANGES = {
    'red1': {'lower': np.array([0, 100, 100]), 'upper': np.array([10, 255, 255]), 'name': '红色', 'color': (0, 0, 255)},
    'red2': {'lower': np.array([160, 100, 100]), 'upper': np.array([180, 255, 255]), 'name': '红色', 'color': (0, 0, 255)},
    'green': {'lower': np.array([35, 100, 100]), 'upper': np.array([85, 255, 255]), 'name': '绿色', 'color': (0, 255, 0)},
    'blue': {'lower': np.array([100, 100, 100]), 'upper': np.array([130, 255, 255]), 'name': '蓝色', 'color': (255, 0, 0)},
    'yellow': {'lower': np.array([20, 100, 100]), 'upper': np.array([35, 255, 255]), 'name': '黄色', 'color': (0, 255, 255)},
    'orange': {'lower': np.array([10, 100, 100]), 'upper': np.array([20, 255, 255]), 'name': '橙色', 'color': (0, 165, 255)}
}

def detect_colors(frame):
    """
    检测图像中的颜色并用方框标记
    
    参数:
        frame: 输入的图像帧
        
    返回:
        标记了颜色区域的图像帧
    """
    # 转换到HSV颜色空间
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # 创建结果图像的副本
    result = frame.copy()
    
    # 检测每种颜色
    for color_name, color_info in COLOR_RANGES.items():
        # 创建颜色掩码
        mask = cv2.inRange(hsv, color_info['lower'], color_info['upper'])
        
        # 应用形态学操作来去除噪点
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # 查找轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # 绘制轮廓和标签
        for contour in contours:
            # 计算轮廓面积，过滤掉太小的区域
            area = cv2.contourArea(contour)
            if area > 500:  # 最小面积阈值
                # 获取边界矩形
                x, y, w, h = cv2.boundingRect(contour)
                
                # 绘制矩形
                cv2.rectangle(result, (x, y), (x + w, y + h), color_info['color'], 2)
                
                # 添加文本标签
                cv2.putText(result, color_info['name'], (x, y - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, color_info['color'], 2)
    
    return result

def start_color_recognition():
    """
    启动颜色识别功能
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
        
        # 等待3秒
        time.sleep(3)
        
        # 启动颜色识别节点
        cmd2 = "rosrun limo_visions recognition"
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

def start_color_tracking():
    """
    启动颜色跟踪功能
    """
    processes = []
    
    try:
        # 启动颜色跟踪launch文件
        env = os.environ.copy()
        env["ROS_MASTER_URI"] = "http://master:11311"
        
        cmd = "roslaunch limo_visions follow.launch"
        p = subprocess.Popen(cmd, shell=True, env=env, preexec_fn=os.setsid)
        processes.append(p)
        print(f"已启动颜色跟踪: {cmd}")
        
        # 等待进程结束或被终止
        while all(p.poll() is None for p in processes):
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("接收到终止信号，正在停止颜色跟踪...")
    except Exception as e:
        print(f"启动颜色跟踪时发生错误: {e}")
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
        
        print("颜色跟踪已停止")

def stop_color_recognition():
    """
    停止颜色识别功能
    """
    try:
        # 查找相关进程
        cmd = "ps aux | grep -E 'module18_color_recognition.py|limo_visions|astra_camera' | grep -v grep"
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
            except Exception as e:
                print(f"发送终止信号到进程 {pid} 时出错: {e}")
        
        # 关闭所有OpenCV窗口
        try:
            subprocess.run("pkill -f 'python.*module18_color_recognition.py'", shell=True)
            subprocess.run("pkill -f 'rosrun limo_visions'", shell=True)
            subprocess.run("pkill -f 'roslaunch limo_visions'", shell=True)
            subprocess.run("pkill -f 'roslaunch astra_camera'", shell=True)
        except:
            pass
            
        print("颜色识别已停止")
        
    except Exception as e:
        print(f"停止颜色识别时出错: {e}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("用法: python3 module18_color_recognition.py [start|track|stop]")
        sys.exit(1)
        
    action = sys.argv[1]
    
    if action == "start":
        start_color_recognition()
    elif action == "track":
        start_color_tracking()
    elif action == "stop":
        stop_color_recognition()
    else:
        print(f"未知操作: {action}")
        print("用法: python3 module18_color_recognition.py [start|track|stop]")
        sys.exit(1)
