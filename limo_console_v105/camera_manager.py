#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
摄像头视频显示模块
用于LIMO控制台中的视频显示和摄像头测试功能
"""

import cv2
import numpy as np
import threading
import time
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt5.QtGui import QImage, QPixmap

class CameraManager(QObject):
    """摄像头管理类，负责摄像头的打开、关闭和视频帧获取"""
    
    # 定义信号
    frame_ready = pyqtSignal(QImage)
    error_occurred = pyqtSignal(str)
    
    def __init__(self, parent=None):
        super(CameraManager, self).__init__(parent)
        
        # 摄像头相关变量
        self.camera = None
        self.camera_id = 0
        self.running = False
        self.capture_thread = None
        self.frame_width = 640
        self.frame_height = 480
        self.frame_rate = 30
    
    def start_camera(self, camera_id=0):
        """启动摄像头"""
        if self.running:
            self.error_occurred.emit("摄像头已经在运行中")
            return False
        
        try:
            # 确保之前的资源已经完全释放
            if self.camera is not None:
                try:
                    self.camera.release()
                except:
                    pass
                self.camera = None
                
            # 尝试释放可能被占用的摄像头资源
            try:
                temp_cap = cv2.VideoCapture(camera_id)
                temp_cap.release()
                time.sleep(0.5)  # 等待资源释放
            except:
                pass
            
            # 打开摄像头
            self.camera_id = camera_id
            self.camera = cv2.VideoCapture(camera_id)
            
            # 设置分辨率和帧率
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
            self.camera.set(cv2.CAP_PROP_FPS, self.frame_rate)
            
            # 检查摄像头是否成功打开
            if not self.camera.isOpened():
                # 如果打开失败，尝试其他设备ID
                alternative_ids = [0, 2, 1, 3, 4]  # 常见的摄像头设备ID
                for alt_id in alternative_ids:
                    if alt_id == camera_id:
                        continue  # 跳过已尝试的ID
                    
                    try:
                        self.camera.release()  # 释放之前的资源
                        self.camera = cv2.VideoCapture(alt_id)
                        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
                        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
                        self.camera.set(cv2.CAP_PROP_FPS, self.frame_rate)
                        
                        if self.camera.isOpened():
                            self.camera_id = alt_id
                            print(f"成功使用替代设备ID {alt_id} 打开摄像头")
                            break
                    except:
                        continue
            
            # 再次检查摄像头是否成功打开
            if not self.camera.isOpened():
                self.error_occurred.emit(f"无法打开摄像头 {camera_id}，已尝试多个设备ID")
                return False
            
            # 启动捕获线程
            self.running = True
            self.capture_thread = threading.Thread(target=self._capture_loop)
            self.capture_thread.daemon = True
            self.capture_thread.start()
            
            return True
        
        except Exception as e:
            self.error_occurred.emit(f"启动摄像头时出现问题: {str(e)}")
            return False
    
    def stop_camera(self):
        """停止摄像头"""
        self.running = False
        
        # 等待捕获线程结束
        if self.capture_thread and self.capture_thread.is_alive():
            self.capture_thread.join(timeout=1.0)
        
        # 释放摄像头资源
        if self.camera:
            try:
                self.camera.release()
                self.camera = None
                # 强制执行垃圾回收，确保资源被释放
                import gc
                gc.collect()
                time.sleep(0.5)  # 给系统一些时间来释放资源
            except Exception as e:
                print(f"释放摄像头资源时出错: {str(e)}")
                # 即使出错也设置为None，避免后续使用已损坏的对象
                self.camera = None
    
    def _capture_loop(self):
        """视频捕获循环"""
        while self.running:
            try:
                # 读取一帧
                ret, frame = self.camera.read()
                
                if not ret:
                    self.error_occurred.emit("读取视频帧未成功，请检查摄像头连接")
                    self.running = False
                    break
                
                # 转换为RGB格式（OpenCV使用BGR格式）
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                
                # 创建QImage
                h, w, ch = frame_rgb.shape
                bytes_per_line = ch * w
                qt_image = QImage(frame_rgb.data, w, h, bytes_per_line, QImage.Format_RGB888)
                
                # 发送信号
                self.frame_ready.emit(qt_image)
                
                # 控制帧率
                time.sleep(1.0 / self.frame_rate)
            
            except Exception as e:
                self.error_occurred.emit(f"视频捕获过程中出现问题: {str(e)}")
                self.running = False
                break
    
    def is_running(self):
        """检查摄像头是否正在运行"""
        return self.running
