#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LIMO控制台语言管理模块
负责管理中英文翻译和语言切换
"""

class LanguageManager:
    """语言管理类，负责处理中英文翻译"""
    
    def __init__(self):
        # 默认语言为中文
        self.current_language = "zh_CN"
        
        # 初始化翻译字典
        self._init_translations()
    
    def _init_translations(self):
        """初始化翻译字典"""
        self.translations = {
            # 窗口标题
            "window_title": {
                "zh_CN": "LIMO控制台",
                "en_US": "LIMO Console"
            },
            
            # 状态栏消息
            "status_started": {
                "zh_CN": "LIMO控制台已启动",
                "en_US": "LIMO Console started"
            },
            "status_running": {
                "zh_CN": "正在运行: {}",
                "en_US": "Running: {}"
            },
            "status_stopped": {
                "zh_CN": "已停止: {}",
                "en_US": "Stopped: {}"
            },
            "status_reset": {
                "zh_CN": "已复位所有功能",
                "en_US": "All functions reset"
            },
            
            # 运动模式
            "motion_mode_0": {
                "zh_CN": "差速模式",
                "en_US": "Differential Mode"
            },
            "motion_mode_1": {
                "zh_CN": "阿克曼模式",
                "en_US": "Ackermann Mode"
            },
            "motion_mode_2": {
                "zh_CN": "麦轮模式",
                "en_US": "Mecanum Mode"
            },
            "motion_mode_3": {
                "zh_CN": "全向模式",
                "en_US": "Omni Mode"
            },
            
            # 控制模式
            "control_mode_0": {
                "zh_CN": "遥控器模式",
                "en_US": "Remote Control Mode"
            },
            "control_mode_1": {
                "zh_CN": "自动驾驶模式",
                "en_US": "Autonomous Mode"
            },
            
            # 错误代码
            "error_code_0x0001": {
                "zh_CN": "电池电压低",
                "en_US": "Low Battery Voltage"
            },
            "error_code_0x0002": {
                "zh_CN": "IMU错误",
                "en_US": "IMU Error"
            },
            "error_code_0x0004": {
                "zh_CN": "里程计错误",
                "en_US": "Odometry Error"
            },
            "error_code_0x0008": {
                "zh_CN": "电机驱动错误",
                "en_US": "Motor Driver Error"
            },
            "error_code_0x0010": {
                "zh_CN": "通信超时",
                "en_US": "Communication Timeout"
            },
            "error_code_0x0020": {
                "zh_CN": "紧急停止",
                "en_US": "Emergency Stop"
            },
            
            # 按钮文本
            "btn_language": {
                "zh_CN": "English",
                "en_US": "中文"
            },
        }
    
    def get_text(self, key, default=None):
        """获取指定键的翻译文本"""
        if key in self.translations:
            return self.translations[key].get(self.current_language, default)
        return default if default is not None else key
    
    def toggle_language(self):
        """切换语言"""
        if self.current_language == "zh_CN":
            self.current_language = "en_US"
        else:
            self.current_language = "zh_CN"
        return self.current_language
    
    def get_current_language(self):
        """获取当前语言"""
        return self.current_language
