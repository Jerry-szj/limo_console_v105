#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LIMO控制台启动脚本
检查依赖并启动控制台
"""

import os
import sys
import subprocess
import importlib.util
import time

def check_dependencies():
    """检查依赖项"""
    print("正在检查依赖项...")
    
    # 检查Python版本
    python_version = sys.version_info
    print(f"Python版本: {python_version.major}.{python_version.minor}.{python_version.micro}")
    if python_version.major < 3 or (python_version.major == 3 and python_version.minor < 6):
        print("错误: 需要Python 3.6或更高版本")
        return False
    
    # 检查必要的Python包
    required_packages = ["PyQt5", "numpy", "matplotlib", "opencv-python"]
    missing_packages = []
    
    for package in required_packages:
        try:
            importlib.import_module(package.replace("-", "_"))
            print(f"✓ {package} 已安装")
        except ImportError:
            print(f"✗ {package} 未安装")
            missing_packages.append(package)
    
    # 如果有缺失的包，尝试安装
    if missing_packages:
        print("\n需要安装以下包:")
        for package in missing_packages:
            print(f"  - {package}")
        
        try:
            print("\n正在安装缺失的包...")
            subprocess.check_call([sys.executable, "-m", "pip", "install"] + missing_packages)
            print("所有依赖包已成功安装")
        except subprocess.CalledProcessError:
            print("错误: 无法安装依赖包")
            return False
    
    # 检查pylimo库
    try:
        sys.path.append('/home/ubuntu/upload')
        import pylimo
        print("✓ pylimo库可用")
    except ImportError:
        print("警告: pylimo库不可用，将无法使用pylimo模式读取底盘数据")
        print("请确保pylimo库位于/home/ubuntu/upload目录下")
    
    # 检查ROS环境
    try:
        ros_distro = os.environ.get("ROS_DISTRO")
        if ros_distro:
            print(f"✓ ROS环境可用 (ROS {ros_distro})")
        else:
            print("警告: ROS环境变量未设置，可能无法使用ROS功能")
    except Exception:
        print("警告: 无法检测ROS环境")
    
    return True

def start_console():
    """启动控制台"""
    print("\n正在启动LIMO控制台...")
    
    # 获取当前脚本所在目录
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    # 构建控制台脚本路径
    console_script = os.path.join(script_dir, "new_limo_console.py")
    
    # 检查控制台脚本是否存在
    if not os.path.exists(console_script):
        console_script = os.path.join(script_dir, "limo_console.py")
        if not os.path.exists(console_script):
            print(f"错误: 控制台脚本不存在: {console_script}")
            return False
    
    try:
        # 启动控制台
        subprocess.run([sys.executable, console_script])
        return True
    except Exception as e:
        print(f"启动控制台时出错: {str(e)}")
        return False

if __name__ == "__main__":
    print("=" * 50)
    print("LIMO控制台启动程序")
    print("=" * 50)
    
    # 检查依赖项
    if check_dependencies():
        # 启动控制台
        start_console()
    else:
        print("\n错误: 依赖项检查失败，无法启动控制台")
        sys.exit(1)
