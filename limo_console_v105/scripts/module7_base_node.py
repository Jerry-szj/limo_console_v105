#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
功能模块7：Limo驱动
启动LIMO基础节点
"""

import os
import sys
import time
import signal
import subprocess

def start_base_node():
    """
    启动基础节点
    """
    processes = []
    
    try:
        # 首先确保没有残留的进程
        stop_all()
        
        # 等待一段时间确保所有进程都已停止
        time.sleep(2)
        
        # 切换到工作目录
        os.chdir(os.path.expanduser("~/agilex_ws"))
        
        # 设置环境变量
        env = os.environ.copy()
        # 使用默认的ROS_MASTER_URI，而不是设置为特定值
        # env["ROS_MASTER_URI"] = "http://master:11311"
        
        # 使用bash执行命令，确保source命令生效
        cmd = "source devel/setup.bash && roslaunch limo_bringup limo_start.launch pub_odom_tf:=false"
        p = subprocess.Popen(cmd, shell=True, executable="/bin/bash", env=env, preexec_fn=os.setsid)
        processes.append(p)
        print(f"已启动: {cmd}")
        
        # 等待一段时间确保节点启动完成
        print("等待基础节点启动完成...")
        time.sleep(3)
        
        # 检查节点是否成功启动
        check_cmd = "source devel/setup.bash && rostopic list | grep limo"
        result = subprocess.run(check_cmd, shell=True, executable="/bin/bash", stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        
        if result.returncode == 0 and result.stdout.strip():
            print("基础节点已成功启动")
            # 创建一个标记文件，表示基础节点已启动
            with open("/tmp/limo_base_node_running", "w") as f:
                f.write("running")
        else:
            print("基础节点可能未成功启动，请检查")
            
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
        
        # 删除标记文件
        try:
            os.remove("/tmp/limo_base_node_running")
        except:
            pass
            
        print("所有进程已停止")

def stop_all():
    """
    停止所有相关进程
    """
    try:
        print("正在停止所有ROS相关进程...")
        
        # 首先尝试优雅地关闭limo_base节点
        try:
            # 查找limo_base相关进程
            cmd = "ps aux | grep -E 'limo_base|limo_start' | grep -v grep"
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
                    print(f"正在优雅地终止limo_base进程 {pid}...")
                    os.kill(int(pid), signal.SIGINT)
                except Exception as e:
                    print(f"发送SIGINT到进程 {pid} 时出错: {e}")
            
            # 等待2秒让进程有时间优雅地退出
            time.sleep(2)
            
            # 检查是否还有limo_base进程在运行
            cmd = "ps aux | grep -E 'limo_base|limo_start' | grep -v grep"
            output = subprocess.check_output(cmd, shell=True).decode('utf-8')
            
            # 如果还有进程，强制终止
            if output.strip():
                print("一些limo_base进程仍在运行，正在强制终止...")
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
                        print(f"发送SIGKILL到进程 {pid} 时出错: {e}")
        except Exception as e:
            print(f"终止limo_base进程时出错: {e}")
        
        # 然后终止所有其他ROS相关进程
        try:
            # 查找所有ROS相关进程
            cmd = "ps aux | grep -E 'roslaunch|rosrun|roscore|rosmaster' | grep -v grep"
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
                    print(f"发送SIGINT到进程 {pid} 时出错: {e}")
        except Exception as e:
            print(f"终止ROS进程时出错: {e}")
        
        # 等待3秒
        print("等待进程终止...")
        time.sleep(3)
        
        # 检查是否还有ROS相关进程在运行
        try:
            cmd = "ps aux | grep -E 'roslaunch|rosrun|roscore|rosmaster|limo_base|limo_start' | grep -v grep"
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
                        print(f"发送SIGKILL到进程 {pid} 时出错: {e}")
        except Exception as e:
            print(f"强制终止ROS进程时出错: {e}")
        
        # 关闭所有终端窗口
        try:
            subprocess.run("pkill -f xterm", shell=True)
        except Exception as e:
            print(f"关闭终端窗口时出错: {e}")
        
        # 释放串口资源
        try:
            print("正在释放串口资源...")
            # 查找可能占用串口的进程
            cmd = "lsof /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || true"
            output = subprocess.check_output(cmd, shell=True).decode('utf-8')
            
            # 提取PID
            for line in output.strip().split('\n'):
                if not line or "COMMAND" in line:  # 跳过标题行
                    continue
                
                parts = line.split()
                if len(parts) < 2:
                    continue
                    
                pid = parts[1]
                try:
                    # 发送SIGKILL信号
                    print(f"正在终止占用串口的进程 {pid}...")
                    os.kill(int(pid), signal.SIGKILL)
                except Exception as e:
                    print(f"终止占用串口的进程 {pid} 时出错: {e}")
        except Exception as e:
            print(f"释放串口资源时出错: {e}")
        
        # 删除标记文件
        try:
            os.remove("/tmp/limo_base_node_running")
        except:
            pass
            
        print("所有进程已停止，串口资源已释放")
        
    except Exception as e:
        print(f"停止进程时出错: {e}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("用法: python3 module7_base_node.py [start|stop]")
        sys.exit(1)
        
    action = sys.argv[1]
    
    if action == "start":
        start_base_node()
    elif action == "stop":
        stop_all()
    else:
        print(f"未知操作: {action}")
        print("用法: python3 module7_base_node.py [start|stop]")
        sys.exit(1)
