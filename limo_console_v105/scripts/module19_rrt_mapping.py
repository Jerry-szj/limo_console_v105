#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
功能模块19：RRT建图
实现RRT建图功能，按顺序启动相关节点
"""

import os
import sys
import time
import signal
import subprocess

def start_rrt_mapping():
    """
    启动RRT建图功能
    按顺序启动各个节点，每个命令间隔4秒
    """
    processes = []
    
    try:
        # 切换到工作目录
        os.chdir(os.path.expanduser("~/agilex_ws"))
        
        # 设置环境变量
        env = os.environ.copy()
        
        # 首先运行source命令
        print("正在设置ROS环境...")
        subprocess.run("source devel/setup.bash", shell=True, executable="/bin/bash")
        
        # 启动limo_start.launch
        print("正在启动基础节点...")
        cmd1 = "source devel/setup.bash && roslaunch limo_bringup limo_start.launch pub_odom_tf:=false"
        p1 = subprocess.Popen(cmd1, shell=True, executable="/bin/bash", env=env, preexec_fn=os.setsid)
        processes.append(p1)
        print(f"已启动: {cmd1}")
        
        # 等待4秒
        print("等待基础节点启动完成...")
        time.sleep(4)
        
        # 启动limo_gmapping.launch
        print("正在启动Gmapping...")
        cmd2 = "source devel/setup.bash && roslaunch limo_bringup limo_gmapping.launch"
        p2 = subprocess.Popen(cmd2, shell=True, executable="/bin/bash", env=env, preexec_fn=os.setsid)
        processes.append(p2)
        print(f"已启动: {cmd2}")
        
        # 等待4秒
        print("等待Gmapping启动完成...")
        time.sleep(4)
        
        # 启动limo_move_base.launch
        print("正在启动Move Base...")
        cmd3 = "source devel/setup.bash && roslaunch limo_bringup limo_move_base.launch"
        p3 = subprocess.Popen(cmd3, shell=True, executable="/bin/bash", env=env, preexec_fn=os.setsid)
        processes.append(p3)
        print(f"已启动: {cmd3}")
        
        # 等待4秒
        print("等待Move Base启动完成...")
        time.sleep(4)
        
        # 启动rrt_exploration simple.launch
        print("正在启动RRT Exploration...")
        cmd4 = "source devel/setup.bash && roslaunch rrt_exploration simple.launch"
        p4 = subprocess.Popen(cmd4, shell=True, executable="/bin/bash", env=env, preexec_fn=os.setsid)
        processes.append(p4)
        print(f"已启动: {cmd4}")
        
        print("RRT建图已全部启动")
        
        # 等待进程结束或被终止
        while all(p.poll() is None for p in processes):
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("接收到终止信号，正在停止所有进程...")
    except Exception as e:
        print(f"发生错误: {e}")
    finally:
        # 终止所有进程，按照相反的顺序
        for p in reversed(processes):
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGINT)
                time.sleep(1)
                if p.poll() is None:
                    os.killpg(os.getpgid(p.pid), signal.SIGTERM)
            except:
                pass
        
        print("所有进程已停止")

def stop_rrt_mapping():
    """
    停止RRT建图功能
    按照启动的相反顺序停止各个节点
    """
    try:
        print("正在停止RRT建图...")
        
        # 停止RRT Exploration
        print("正在停止RRT Exploration...")
        subprocess.run("rosnode kill /rrt_exploration", shell=True, stderr=subprocess.DEVNULL)
        time.sleep(1)
        
        # 停止Move Base
        print("正在停止Move Base...")
        subprocess.run("rosnode kill /move_base", shell=True, stderr=subprocess.DEVNULL)
        time.sleep(1)
        
        # 停止Gmapping
        print("正在停止Gmapping...")
        subprocess.run("rosnode kill /slam_gmapping", shell=True, stderr=subprocess.DEVNULL)
        time.sleep(1)
        
        # 停止基础节点
        print("正在停止基础节点...")
        subprocess.run("rosnode kill /limo_base_node", shell=True, stderr=subprocess.DEVNULL)
        
        # 查找并终止所有相关进程
        print("正在终止所有相关进程...")
        cmd = "ps aux | grep -E 'rrt_exploration|move_base|gmapping|limo_start' | grep -v grep"
        output = subprocess.check_output(cmd, shell=True).decode('utf-8')
        
        # 提取PID并终止进程
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
                time.sleep(0.5)
                
                # 检查进程是否仍在运行
                try:
                    os.kill(int(pid), 0)
                    # 如果进程仍在运行，发送SIGKILL信号
                    os.kill(int(pid), signal.SIGKILL)
                    print(f"已强制终止进程 {pid}")
                except OSError:
                    # 进程已经终止
                    pass
            except:
                pass
        
        print("RRT建图已停止")
        
    except Exception as e:
        print(f"停止RRT建图时出错: {e}")

def save_map():
    """
    保存RRT建图生成的地图
    使用与gmapping相同的方式保存地图
    """
    try:
        print("正在保存RRT建图地图...")
        
        # 创建地图保存目录
        map_dir = os.path.join(os.path.expanduser("~"), "agilex_ws/src/limo_ros/limo_bringup/maps")
        os.makedirs(map_dir, exist_ok=True)
        
        # 设置地图文件路径
        map_path = os.path.join(map_dir, "rrt_map")
        
        # 保存地图
        cmd = f"source ~/agilex_ws/devel/setup.bash && rosrun map_server map_saver -f {map_path}"
        result = subprocess.run(cmd, shell=True, executable="/bin/bash", stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        
        if result.returncode == 0:
            print(f"RRT建图地图已保存到: {map_path}")
            return True, f"RRT建图地图已保存到: {map_path}"
        else:
            error_msg = f"保存RRT建图地图失败: {result.stderr}"
            print(error_msg)
            return False, error_msg
    
    except Exception as e:
        error_msg = f"保存RRT建图地图时出错: {str(e)}"
        print(error_msg)
        return False, error_msg

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("用法: python3 module19_rrt_mapping.py [start|stop|save]")
        sys.exit(1)
        
    action = sys.argv[1]
    
    if action == "start":
        start_rrt_mapping()
    elif action == "stop":
        stop_rrt_mapping()
    elif action == "save":
        success, message = save_map()
        if not success:
            sys.exit(1)
    else:
        print(f"未知操作: {action}")
        print("用法: python3 module19_rrt_mapping.py [start|stop|save]")
        sys.exit(1)
