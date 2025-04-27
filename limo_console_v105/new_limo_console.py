#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LIMO控制台主程序 - 重构版
集成状态监控、功能模块控制、视频显示和运动控制
"""

import sys
import os
import time
import threading
import subprocess
from datetime import datetime
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.font_manager as fm

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
    QLabel, QProgressBar, QTextEdit, QGroupBox, QRadioButton, 
    QPushButton, QTabWidget, QStackedWidget, QToolBar, QStatusBar,
    QSplitter, QFrame, QButtonGroup, QGridLayout, QScrollArea,
    QSizePolicy, QSpacerItem, QComboBox
)
from PyQt5.QtCore import Qt, QProcess, QTimer, pyqtSignal, QSize, QPoint
from PyQt5.QtGui import QIcon, QPixmap, QFont, QColor, QPalette, QTextCursor, QTextCharFormat, QImage, QMouseEvent

# 导入自定义模块
from status_monitor import StatusMonitor
from script_executor import ScriptExecutor
from camera_manager import CameraManager
from motion_controller import MotionController

# 配置matplotlib支持中文显示
plt.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans', 'Bitstream Vera Sans', 'sans-serif']
plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题

class MatplotlibCanvas(FigureCanvas):
    """Matplotlib画布类，用于在Qt界面中嵌入matplotlib图表"""
    def __init__(self, parent=None, width=5, height=4, dpi=100):
        self.fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = self.fig.add_subplot(111)
        super(MatplotlibCanvas, self).__init__(self.fig)
        self.setParent(parent)
        
        # 调整图表边距，增加左边距以确保坐标轴数据完全可见
        self.fig.subplots_adjust(left=0.15, right=0.95, bottom=0.15, top=0.9)
        # 不使用tight_layout，因为我们需要精确控制边距

class ModuleButton(QGroupBox):
    """模块按钮类，用于创建统一样式的功能模块按钮"""
    def __init__(self, title, options=None, parent=None):
        super(ModuleButton, self).__init__(title, parent)
        
        # 设置大小策略，使其能够在网格布局中自适应大小
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        
        # 设置样式
        self.setStyleSheet("""
            QGroupBox {
                border: 2px solid #3498db;
                border-radius: 8px;
                margin-top: 15px;
                font-weight: bold;
                background-color: #ecf0f1;
                min-height: 120px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top center;
                padding: 0 8px;
                background-color: #3498db;
                color: white;
                border-radius: 4px;
                font-size: 14px;
            }
        """)
        
        # 创建布局
        self.layout = QVBoxLayout(self)
        self.layout.setContentsMargins(15, 25, 15, 15)
        self.layout.setSpacing(3)
        
        # 添加选项（如果有）
        self.option_combos = {}
        if options:
            for option in options:
                option_layout = QHBoxLayout()
                option_label = QLabel(option["name"] + ":")
                option_label.setStyleSheet("font-weight: normal;")
                option_combo = QComboBox()
                option_combo.addItems(option["values"])
                option_combo.setStyleSheet("""
                    QComboBox {
                        padding: 5px;
                        border: 1px solid #bdc3c7;
                        border-radius: 4px;
                        background-color: white;
                    }
                    QComboBox::drop-down {
                        border: none;
                        width: 20px;
                    }
                    QComboBox QAbstractItemView {
                        selection-background-color: #3498db;
                        selection-color: white;
                        background-color: white;
                        border: 1px solid #bdc3c7;
                        border-radius: 0px;
                    }
                    QComboBox QAbstractItemView::item {
                        min-height: 25px;
                    }
                """)
                self.option_combos[option["name"]] = {
                    "combo": option_combo,
                    "param_values": option["param_values"]
                }
                option_layout.addWidget(option_label)
                option_layout.addWidget(option_combo)
                self.layout.addLayout(option_layout)
        
        # 添加按钮
        self.buttons_layout = QHBoxLayout()
        
        # 开始按钮
        self.start_button = QPushButton("开始")
        self.start_button.setStyleSheet("""
            QPushButton {
                background-color: #2ecc71;
                color: white;
                border-radius: 4px;
                padding: 8px;
                font-weight: bold;
                font-size: 13px;
            }
            QPushButton:hover {
                background-color: #27ae60;
            }
        """)
        self.buttons_layout.addWidget(self.start_button)
        
        # 结束按钮
        self.stop_button = QPushButton("结束")
        self.stop_button.setStyleSheet("""
            QPushButton {
                background-color: #e74c3c;
                color: white;
                border-radius: 4px;
                padding: 8px;
                font-weight: bold;
                font-size: 13px;
            }
            QPushButton:hover {
                background-color: #c0392b;
            }
        """)
        self.buttons_layout.addWidget(self.stop_button)
        
        self.layout.addLayout(self.buttons_layout)
    
    def get_options(self):
        """获取当前选项值"""
        options = {}
        for name, option_data in self.option_combos.items():
            combo = option_data["combo"]
            param_values = option_data["param_values"]
            selected_index = combo.currentIndex()
            options[name] = param_values[selected_index]
        return options

class MouseControlArea(QFrame):
    """鼠标控制区域类，用于捕获鼠标事件并控制速度"""
    
    # 定义信号
    position_changed = pyqtSignal(float, float)
    
    def __init__(self, parent=None):
        super(MouseControlArea, self).__init__(parent)
        
        # 设置框架样式
        self.setFrameShape(QFrame.StyledPanel)
        self.setStyleSheet("""
            QFrame {
                background-color: #ecf0f1;
                border: 1px solid #bdc3c7;
                border-radius: 4px;
                min-height: 120px;
            }
        """)
        
        # 启用鼠标跟踪
        self.setMouseTracking(True)
        
        # 鼠标状态
        self.mouse_pressed = False
        self.center_point = QPoint(0, 0)
        self.current_point = QPoint(0, 0)
        
        # 创建布局
        self.layout = QVBoxLayout(self)
        
        # 添加标签
        self.info_label = QLabel("鼠标控制区域\n(点击并拖动鼠标控制速度)")
        self.info_label.setAlignment(Qt.AlignCenter)
        self.layout.addWidget(self.info_label)
        
        # 添加速度显示标签
        self.speed_label = QLabel("线速度: 0.00 m/s  角速度: 0.00 rad/s")
        self.speed_label.setAlignment(Qt.AlignCenter)
        self.layout.addWidget(self.speed_label)
    
    def resizeEvent(self, event):
        """重写调整大小事件，更新中心点"""
        super().resizeEvent(event)
        self.center_point = QPoint(self.width() // 2, self.height() // 2)
    
    def mousePressEvent(self, event):
        """鼠标按下事件"""
        if event.button() == Qt.LeftButton:
            self.mouse_pressed = True
            self.current_point = event.pos()
            self.update_position()
    
    def mouseReleaseEvent(self, event):
        """鼠标释放事件"""
        if event.button() == Qt.LeftButton:
            self.mouse_pressed = False
            self.current_point = self.center_point
            self.update_position()
    
    def mouseMoveEvent(self, event):
        """鼠标移动事件"""
        if self.mouse_pressed:
            self.current_point = event.pos()
            self.update_position()
    
    def update_position(self):
        """更新位置并发送信号"""
        if not self.mouse_pressed:
            x_ratio = 0.0
            y_ratio = 0.0
        else:
            # 计算相对于中心点的位置比例
            x_diff = self.current_point.x() - self.center_point.x()
            y_diff = self.center_point.y() - self.current_point.y()  # 反转Y轴，向上为正
            
            # 计算最大距离（取宽度和高度的最小值的一半）
            max_distance = min(self.width(), self.height()) / 2
            
            # 计算比例，范围限制在[-1, 1]
            x_ratio = max(-1.0, min(1.0, x_diff / max_distance))
            y_ratio = max(-1.0, min(1.0, y_diff / max_distance))
        
        # 更新速度显示
        linear_speed = y_ratio * 0.5  # 最大线速度0.5 m/s
        angular_speed = -x_ratio * 1.0  # 最大角速度1.0 rad/s
        self.speed_label.setText(f"线速度: {linear_speed:.2f} m/s  角速度: {angular_speed:.2f} rad/s")
        
        # 发送信号
        self.position_changed.emit(x_ratio, y_ratio)

class LimoConsole(QMainWindow):
    """LIMO控制台主窗口类"""
    def __init__(self):
        super(LimoConsole, self).__init__()
        
        # 设置窗口属性
        self.setWindowTitle("LIMO控制台")
        self.setMinimumSize(1200, 800)
        
        # 初始化状态监控器
        self.status_monitor = StatusMonitor(self)
        self.status_monitor.status_updated.connect(self.update_status_display)
        
        # 初始化脚本执行器
        self.script_executor = ScriptExecutor(self)
        self.script_executor.output_received.connect(self.append_output)
        self.script_executor.error_received.connect(self.append_error)
        
        # 初始化摄像头管理器
        self.camera_manager = CameraManager(self)
        self.camera_manager.frame_ready.connect(self.update_video_frame)
        self.camera_manager.error_occurred.connect(self.append_error)
        
        # 初始化运动控制器
        self.motion_controller = MotionController()
        
        # 定义功能模块分类
        self._define_module_categories()
        
        # 创建界面
        self._create_ui()
        
        # 创建定时器，用于更新图表
        self.chart_timer = QTimer(self)
        self.chart_timer.timeout.connect(self.update_charts)
        self.chart_timer.start(1000)  # 每秒更新一次图表
        
        # 创建定时器，用于检测roscore状态
        self.roscore_timer = QTimer(self)
        self.roscore_timer.timeout.connect(self.check_roscore_status)
        self.roscore_timer.start(5000)  # 每5秒检测一次roscore状态
        
        # 显示状态栏消息
        self.statusBar().showMessage("LIMO控制台已启动")
    
    def _define_module_categories(self):
        """定义功能模块分类"""
        # 定义所有功能模块
        self.all_modules = [
            {
                "name": "Gmapping建图", 
                "module_id": 1,
                "options": None,
                "categories": ["SLAM建图"]
            },
            {
                "name": "Cartographer建图", 
                "module_id": 2,
                "options": None,
                "categories": ["SLAM建图"]
            },
            {
                "name": "Tmini Pro", 
                "module_id": 3,
                "options": None,
                "categories": ["基础功能"]
            },
            {
                "name": "雷达导航", 
                "module_id": 4,
                "options": [
                    {"name": "底盘模式", "values": ["差速", "阿克曼", "麦轮"], "param_values": ["diff", "ackerman", "mec"]}
                ],
                "categories": ["自主导航"]
            },
            {
                "name": "Limo驱动", 
                "module_id": 7,
                "options": None,
                "categories": ["基础功能"]
            },
            {
                "name": "路径巡检", 
                "module_id": 8,
                "options": [
                    {"name": "模式选择", "values": ["录制路径", "巡检模式"], "param_values": ["record", "follow"]}
                ],
                "categories": ["自主导航"]
            },
            {
                "name": "Dabai DC1", 
                "module_id": 9,
                "options": None,
                "categories": ["基础功能", "视觉模块"]
            },
            {
                "name": "rtabmap建图", 
                "module_id": 10,
                "options": None,
                "categories": ["SLAM建图"]
            },
            {
                "name": "视觉导航", 
                "module_id": 11,
                "options": None,
                "categories": ["自主导航"]
            },
            {
                "name": "文字识别", 
                "module_id": 12,
                "options": None,
                "categories": ["视觉模块"]
            },
            {
                "name": "语音转文字", 
                "module_id": 14,
                "options": None,
                "categories": []
            },
            {
                "name": "语音控制", 
                "module_id": 15,
                "options": None,
                "categories": []
            },
            {
                "name": "RViz", 
                "module_id": 16,
                "options": None,
                "categories": ["基础功能"]
            },
            {
                "name": "RRT建图", 
                "module_id": 19,
                "options": None,
                "categories": ["SLAM建图"]
            },
            {
                "name": "颜色识别", 
                "module_id": 18,
                "options": None,
                "categories": ["视觉模块"]
            },
            {
                "name": "雷达跟随", 
                "module_id": 20,
                "options": None,
                "categories": ["自主导航"]
            },
            {
                "name": "视觉巡线", 
                "module_id": 21,
                "options": None,
                "categories": ["视觉模块"]
            },
            {
                "name": "红绿灯识别", 
                "module_id": 22,
                "options": None,
                "categories": ["视觉模块"]
            },
            {
                "name": "YOLO", 
                "module_id": 23,
                "options": None,
                "categories": ["视觉模块"]
            }
        ]
        
        # 定义分类
        self.categories = ["基础功能", "SLAM建图", "自主导航", "视觉模块"]
        
        # 按分类组织模块
        self.modules_by_category = {}
        for category in self.categories:
            self.modules_by_category[category] = []
            for module in self.all_modules:
                if category in module.get("categories", []):
                    self.modules_by_category[category].append(module)
    
    def _create_ui(self):
        """创建用户界面"""
        # 创建中央部件
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # 创建主布局
        main_layout = QVBoxLayout(central_widget)
        
        # 创建工具栏
        self._create_toolbar()
        
        # 创建主分割器
        self.main_splitter = QSplitter(Qt.Horizontal)
        main_layout.addWidget(self.main_splitter)
        
        # 创建左侧面板
        left_widget = self._create_left_panel()
        self.main_splitter.addWidget(left_widget)
        
        # 创建右侧内容区
        self.right_splitter = QSplitter(Qt.Vertical)
        self.main_splitter.addWidget(self.right_splitter)
        
        # 创建右上状态显示区
        right_top_widget = self._create_status_panel()
        self.right_splitter.addWidget(right_top_widget)
        
        # 创建右下命令行状态区
        right_bottom_widget = self._create_command_panel()
        self.right_splitter.addWidget(right_bottom_widget)
        
        # 设置分割器比例
        self.main_splitter.setSizes([600, 600])  # 左右比例1:1
        self.right_splitter.setSizes([400, 400])  # 上下比例1:1
        
        # 创建状态栏
        self.statusBar()
    
    def _create_toolbar(self):
        """创建工具栏"""
        toolbar = QToolBar("主工具栏")
        toolbar.setMovable(False)
        toolbar.setIconSize(QSize(32, 32))
        self.addToolBar(toolbar)
        
        # 创建一个容器QWidget
        container_widget = QWidget()
        container_layout = QHBoxLayout(container_widget)
        
        # 添加左侧伸缩器
        left_spacer = QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)
        container_layout.addItem(left_spacer)
        
        # 添加标题标签
        title_label = QLabel("LIMO控制台")
        title_label.setFont(QFont("Arial", 16, QFont.Bold))
        title_label.setAlignment(Qt.AlignCenter)  # 设置文字居中
        container_layout.addWidget(title_label)
        
        # 添加右侧伸缩器
        right_spacer = QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)
        container_layout.addItem(right_spacer)
        
        # 将容器添加到工具栏
        toolbar.addWidget(container_widget)
        
        # 添加公司图标（加载真实图标）
        logo_label = QLabel()
        logo_path = "111.png"  # 替换为你的图标文件路径
        logo_pixmap = QPixmap(logo_path)
        
        if not logo_pixmap.isNull():
            print(f"成功加载图标: {logo_path}")
            logo_label.setPixmap(logo_pixmap.scaled(505/3, 164/3, Qt.KeepAspectRatio))  # 调整图标大小
        else:
            print(f"图标文件加载失败，请检查路径是否正确: {logo_path}")
            logo_pixmap = QPixmap(32, 32)
            logo_pixmap.fill(Qt.red)  # 如果加载失败，显示红色方块作为错误提示
            logo_label.setPixmap(logo_pixmap)
        toolbar.addWidget(logo_label)
    
    def _create_left_panel(self):
        """创建左侧面板（分为上下两部分）"""
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        left_layout.setContentsMargins(0, 0, 0, 0)
        
        # 创建左侧分割器（上下分割）
        self.left_splitter = QSplitter(Qt.Vertical)
        left_layout.addWidget(self.left_splitter)
        
        # 创建左上部分（分类标签式功能模块区）
        left_top_widget = self._create_module_tabs()
        self.left_splitter.addWidget(left_top_widget)
        
        # 创建左下部分（视频显示和控制区）
        left_bottom_widget = self._create_control_panel()
        self.left_splitter.addWidget(left_bottom_widget)
        
        # 设置分割比例
        self.left_splitter.setSizes([600, 400])  # 上下比例3:2
        
        return left_widget
    
    def _create_module_tabs(self):
        """创建分类标签式功能模块区"""
        # 创建标签页控件
        self.tab_widget = QTabWidget()
        self.tab_widget.setStyleSheet("""
            QTabWidget::pane {
                border: 1px solid #cccccc;
                background: white;
                border-radius: 5px;
            }
            QTabBar::tab {
                background: #f0f0f0;
                border: 1px solid #cccccc;
                border-bottom: none;
                border-top-left-radius: 4px;
                border-top-right-radius: 4px;
                padding: 8px 12px;
                margin-right: 2px;
            }
            QTabBar::tab:selected {
                background: #3498db;
                color: white;
                font-weight: bold;
            }
            QTabBar::tab:hover:!selected {
                background: #d0d0d0;
            }
        """)
        
        # 创建模块按钮字典，用于存储所有创建的模块按钮
        self.module_buttons = {}
        
        # 为每个分类创建标签页
        for category in self.categories:
            # 创建标签页内容
            tab_content = QWidget()
            tab_layout = QVBoxLayout(tab_content)
            
            # 创建滚动区域
            scroll_area = QScrollArea()
            scroll_area.setWidgetResizable(True)
            scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
            scroll_area.setStyleSheet("border: none;")
            
            # 创建滚动区域内容
            scroll_content = QWidget()
            scroll_layout = QGridLayout(scroll_content)
            scroll_layout.setSpacing(15)
            
            # 获取该分类下的模块
            modules = self.modules_by_category[category]
            
            # 创建模块按钮
            for i, module in enumerate(modules):
                # 如果是占位模块，添加特殊样式
                is_placeholder = module.get("is_placeholder", False)
                
                # 创建模块按钮
                module_button = ModuleButton(module["name"], module["options"])
                
                # 如果是占位模块，禁用按钮并添加特殊样式
                if is_placeholder:
                    module_button.setEnabled(False)
                    module_button.setStyleSheet("""
                        QGroupBox {
                            border: 2px dashed #999999;
                            border-radius: 8px;
                            margin-top: 15px;
                            font-weight: bold;
                            background-color: #f5f5f5;
                            min-height: 120px;
                            color: #999999;
                        }
                        QGroupBox::title {
                            subcontrol-origin: margin;
                            subcontrol-position: top center;
                            padding: 0 8px;
                            background-color: #999999;
                            color: white;
                            border-radius: 4px;
                            font-size: 14px;
                        }
                    """)
                    module_button.start_button.setEnabled(False)
                    module_button.stop_button.setEnabled(False)
                else:
                    # 连接信号
                    module_button.start_button.clicked.connect(
                        lambda checked, m=module: self.start_module(m)
                    )
                    # module_button.stop_button.clicked.connect(
                    #     lambda checked, m=module: self.stop_module(m)
                    # )
                    module_button.stop_button.clicked.connect(self.reset_all)
            
                
                # 计算行和列位置
                row = i // 2  # 每行两个模块
                col = i % 2   # 列索引为0或1
                
                # 添加到布局
                scroll_layout.addWidget(module_button, row, col)
                
                # 存储模块按钮
                if module["module_id"] not in self.module_buttons:
                    self.module_buttons[module["module_id"]] = []
                self.module_buttons[module["module_id"]].append(module_button)
            
            # 添加伸缩器
            scroll_layout.setRowStretch(len(modules) // 2 + 1, 1)
            
            # 设置滚动区域内容
            scroll_area.setWidget(scroll_content)
            
            # 添加到标签页布局
            tab_layout.addWidget(scroll_area)
            
            # 添加标签页
            self.tab_widget.addTab(tab_content, category)
        
        return self.tab_widget
    
    def _create_control_panel(self):
        """创建左下部分（视频显示和控制区）"""
        # 创建容器部件
        control_panel = QWidget()
        control_layout = QHBoxLayout(control_panel)
        control_layout.setContentsMargins(0, 0, 0, 0)
        
        # 创建左右分割器
        control_splitter = QSplitter(Qt.Horizontal)
        control_layout.addWidget(control_splitter)
        
        # 创建左侧视频显示区
        video_widget = self._create_video_panel()
        control_splitter.addWidget(video_widget)
        
        # 创建右侧运动控制区
        motion_widget = self._create_motion_panel()
        control_splitter.addWidget(motion_widget)
        
        # 设置分割比例
        control_splitter.setSizes([300, 300])  # 左右比例1:1
        
        return control_panel
    
    def _create_video_panel(self):
        """创建视频显示面板"""
        # 创建视频显示组
        video_group = QGroupBox("视频显示")
        video_group.setStyleSheet("""
            QGroupBox {
                border: 2px solid #9b59b6;
                border-radius: 8px;
                margin-top: 10px;
                font-weight: bold;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top center;
                padding: 0 5px;
                background-color: #9b59b6;
                color: white;
                border-radius: 3px;
            }
        """)
        
        video_layout = QVBoxLayout(video_group)
        
        # 创建视频显示标签
        self.video_label = QLabel("未启动摄像头")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setStyleSheet("""
            background-color: #2c3e50;
            color: white;
            border-radius: 4px;
            min-height: 240px;
        """)
        video_layout.addWidget(self.video_label)
        
        # 创建测试摄像头按钮
        self.test_camera_button = QPushButton("测试摄像头")
        self.test_camera_button.setStyleSheet("""
            QPushButton {
                background-color: #9b59b6;
                color: white;
                border-radius: 4px;
                padding: 8px;
                font-weight: bold;
                font-size: 13px;
            }
            QPushButton:hover {
                background-color: #8e44ad;
            }
        """)
        self.test_camera_button.clicked.connect(self.toggle_camera_test)
        video_layout.addWidget(self.test_camera_button)
        
        # 摄像头状态标志
        self.camera_active = False
        
        return video_group
    
    def _create_motion_panel(self):
        """创建运动控制面板"""
        # 创建运动控制组
        motion_group = QGroupBox("运动控制")
        motion_group.setStyleSheet("""
            QGroupBox {
                border: 2px solid #e67e22;
                border-radius: 8px;
                margin-top: 10px;
                font-weight: bold;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top center;
                padding: 0 5px;
                background-color: #e67e22;
                color: white;
                border-radius: 3px;
            }
        """)
        
        motion_layout = QVBoxLayout(motion_group)
        
        # 创建方向按钮区域
        direction_widget = QWidget()
        direction_layout = QGridLayout(direction_widget)
        
        # 创建方向按钮
        self.up_button = QPushButton("↑")
        self.down_button = QPushButton("↓")
        self.left_button = QPushButton("←")
        self.right_button = QPushButton("→")
        self.stop_button = QPushButton("■")
        
        # 设置方向按钮样式
        direction_button_style = """
            QPushButton {
                background-color: #3498db;
                color: white;
                border-radius: 4px;
                padding: 10px;
                font-weight: bold;
                font-size: 16px;
                min-width: 40px;
                min-height: 40px;
            }
            QPushButton:hover {
                background-color: #2980b9;
            }
            QPushButton:pressed {
                background-color: #1c6ea4;
            }
        """
        self.up_button.setStyleSheet(direction_button_style)
        self.down_button.setStyleSheet(direction_button_style)
        self.left_button.setStyleSheet(direction_button_style)
        self.right_button.setStyleSheet(direction_button_style)
        
        # 设置停止按钮样式
        self.stop_button.setStyleSheet("""
            QPushButton {
                background-color: #e74c3c;
                color: white;
                border-radius: 4px;
                padding: 10px;
                font-weight: bold;
                font-size: 16px;
                min-width: 40px;
                min-height: 40px;
            }
            QPushButton:hover {
                background-color: #c0392b;
            }
            QPushButton:pressed {
                background-color: #a33025;
            }
        """)
        
        # 添加方向按钮到布局
        direction_layout.addWidget(self.up_button, 0, 1)
        direction_layout.addWidget(self.left_button, 1, 0)
        direction_layout.addWidget(self.stop_button, 1, 1)
        direction_layout.addWidget(self.right_button, 1, 2)
        direction_layout.addWidget(self.down_button, 2, 1)
        
        # 添加方向按钮区域到主布局
        motion_layout.addWidget(direction_widget)
        
        # 创建鼠标控制区域
        self.mouse_control_area = MouseControlArea()
        
        # 添加鼠标控制区域到主布局
        motion_layout.addWidget(self.mouse_control_area)
        
        # 创建激活控制按钮
        self.activate_control_button = QPushButton("激活控制")
        self.activate_control_button.setStyleSheet("""
            QPushButton {
                background-color: #2ecc71;
                color: white;
                border-radius: 4px;
                padding: 8px;
                font-weight: bold;
                font-size: 13px;
            }
            QPushButton:hover {
                background-color: #27ae60;
            }
            QPushButton:checked {
                background-color: #e74c3c;
            }
            QPushButton:checked:hover {
                background-color: #c0392b;
            }
        """)
        self.activate_control_button.setCheckable(True)
        self.activate_control_button.toggled.connect(self.toggle_motion_control)
        motion_layout.addWidget(self.activate_control_button)
        
        # 连接方向按钮信号
        self.up_button.pressed.connect(self.on_up_button_pressed)
        self.up_button.released.connect(self.on_direction_button_released)
        self.down_button.pressed.connect(self.on_down_button_pressed)
        self.down_button.released.connect(self.on_direction_button_released)
        self.left_button.pressed.connect(self.on_left_button_pressed)
        self.left_button.released.connect(self.on_direction_button_released)
        self.right_button.pressed.connect(self.on_right_button_pressed)
        self.right_button.released.connect(self.on_direction_button_released)
        self.stop_button.clicked.connect(self.on_stop_button_clicked)
        
        # 连接鼠标控制区域信号
        self.mouse_control_area.position_changed.connect(self.on_mouse_position_changed)
        
        # 控制状态标志
        self.motion_control_active = False
        
        return motion_group
    
    def _create_status_panel(self):
        """创建右上状态显示面板"""
        status_widget = QWidget()
        status_layout = QVBoxLayout(status_widget)
        
        # 创建状态显示组
        status_group = QGroupBox("LIMO状态")
        status_group.setStyleSheet("""
            QGroupBox {
                border: 2px solid #f39c12;
                border-radius: 8px;
                margin-top: 10px;
                font-weight: bold;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top center;
                padding: 0 5px;
                background-color: #f39c12;
                color: white;
                border-radius: 3px;
            }
        """)
        
        status_inner_layout = QVBoxLayout(status_group)
        
        # 创建roscore状态指示器
        roscore_layout = QHBoxLayout()
        roscore_label = QLabel("ROS核心状态:")
        self.roscore_status_label = QLabel("未运行")
        self.roscore_status_label.setStyleSheet("color: red; font-weight: bold;")
        roscore_layout.addWidget(roscore_label)
        roscore_layout.addWidget(self.roscore_status_label)
        roscore_layout.addStretch()
        status_inner_layout.addLayout(roscore_layout)
        
        # 创建状态数据网格
        status_grid = QGridLayout()
        status_grid.setSpacing(10)
        
        # 添加状态项
        status_items = [
            ("电池电压:", "battery_voltage", "V"),
            ("线速度:", "linear_velocity", "m/s"),
            ("角速度:", "angular_velocity", "rad/s"),
            ("横向速度:", "lateral_velocity", "m/s"),
            ("转向角度:", "steering_angle", "°"),
            ("错误代码:", "error_code", "")
        ]
        
        self.status_labels = {}
        for i, (label_text, key, unit) in enumerate(status_items):
            row = i // 2
            col = (i % 2) * 3
            
            label = QLabel(label_text)
            value_label = QLabel("--")
            unit_label = QLabel(unit)
            
            self.status_labels[key] = value_label
            
            status_grid.addWidget(label, row, col)
            status_grid.addWidget(value_label, row, col + 1)
            status_grid.addWidget(unit_label, row, col + 2)
        
        status_inner_layout.addLayout(status_grid)
        
        # 创建图表区域
        charts_layout = QHBoxLayout()
        
        # 电池电压图表
        battery_group = QGroupBox("电池电压")
        battery_layout = QVBoxLayout(battery_group)
        self.battery_canvas = MatplotlibCanvas(width=4, height=2, dpi=100)
        battery_layout.addWidget(self.battery_canvas)
        charts_layout.addWidget(battery_group)
        
        # 速度图表
        velocity_group = QGroupBox("速度")
        velocity_layout = QVBoxLayout(velocity_group)
        self.velocity_canvas = MatplotlibCanvas(width=4, height=2, dpi=100)
        velocity_layout.addWidget(self.velocity_canvas)
        charts_layout.addWidget(velocity_group)
        
        status_inner_layout.addLayout(charts_layout)
        
        # 添加到状态布局
        status_layout.addWidget(status_group)
        
        return status_widget
    
    def _create_command_panel(self):
        """创建右下命令行状态面板"""
        command_widget = QWidget()
        command_layout = QVBoxLayout(command_widget)
        
        # 创建命令输出组
        command_group = QGroupBox("命令输出")
        command_group.setStyleSheet("""
            QGroupBox {
                border: 2px solid #3498db;
                border-radius: 8px;
                margin-top: 10px;
                font-weight: bold;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top center;
                padding: 0 5px;
                background-color: #3498db;
                color: white;
                border-radius: 3px;
            }
        """)
        
        command_inner_layout = QVBoxLayout(command_group)
        
        # 创建输出文本框
        self.output_text = QTextEdit()
        self.output_text.setReadOnly(True)
        self.output_text.setStyleSheet("""
            QTextEdit {
                background-color: #1e272e;
                color: #ecf0f1;
                border-radius: 4px;
                font-family: Consolas, Monaco, monospace;
                font-size: 13px;
                padding: 10px;
            }
        """)
        command_inner_layout.addWidget(self.output_text)
        
        # 创建按钮布局
        buttons_layout = QHBoxLayout()
        
        # 清空按钮
        clear_button = QPushButton("清空输出")
        clear_button.clicked.connect(self.clear_output)
        clear_button.setStyleSheet("""
            QPushButton {
                background-color: #7f8c8d;
                color: white;
                border-radius: 4px;
                padding: 8px;
                font-weight: bold;
                font-size: 13px;
            }
            QPushButton:hover {
                background-color: #95a5a6;
            }
        """)
        buttons_layout.addWidget(clear_button)
        
        # 保存地图按钮
        save_map_button = QPushButton("一键保存地图")
        save_map_button.clicked.connect(self.save_map)
        save_map_button.setStyleSheet("""
            QPushButton {
                background-color: #2ecc71;
                color: white;
                border-radius: 4px;
                padding: 8px;
                font-weight: bold;
                font-size: 13px;
            }
            QPushButton:hover {
                background-color: #27ae60;
            }
        """)
        buttons_layout.addWidget(save_map_button)
        
        # 复位按钮
        reset_button = QPushButton("复位所有")
        reset_button.clicked.connect(self.reset_all)
        reset_button.setStyleSheet("""
            QPushButton {
                background-color: #e74c3c;
                color: white;
                border-radius: 4px;
                padding: 8px;
                font-weight: bold;
                font-size: 13px;
            }
            QPushButton:hover {
                background-color: #c0392b;
            }
        """)
        buttons_layout.addWidget(reset_button)
        
        command_inner_layout.addLayout(buttons_layout)
        
        # 添加到命令布局
        command_layout.addWidget(command_group)
        
        return command_widget
    
    
    def save_map(self):
        """一键保存地图功能"""
        # 检查ROS核心是否运行
        if self.roscore_status_label.text() != "运行中":
            self.append_error("保存地图失败，ROS核心未运行")
            return
            
        self.append_output("开始一键保存地图，正在检测建图类型...")
        
        try:
            # 调用map_saver.py脚本
            script_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "scripts", "map_saver.py")
            
            # 直接执行脚本并等待结果
            result = subprocess.run(
                [sys.executable, script_path, "save"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                check=False
            )
            
            # 处理输出
            if result.stdout:
                for line in result.stdout.splitlines():
                    self.append_output(line)
            
            if result.stderr:
                for line in result.stderr.splitlines():
                    self.append_error(line)
            
            # 更新状态栏
            if result.returncode == 0:
                self.statusBar().showMessage("地图保存成功")
            else:
                self.statusBar().showMessage("地图保存失败")
        
        except Exception as e:
            self.append_error(f"保存地图时出错: {str(e)}")
            self.statusBar().showMessage("地图保存失败")


    
    def toggle_camera_test(self):
        """切换摄像头测试状态"""
        if not self.camera_active:
            # 启动摄像头
            if self.camera_manager.start_camera():
                self.camera_active = True
                self.test_camera_button.setText("停止测试")
                self.append_output("开始测试摄像头...")
            else:
                self.append_error("启动摄像头失败")
        else:
            # 停止摄像头
            self.camera_manager.stop_camera()
            self.camera_active = False
            self.test_camera_button.setText("测试摄像头")
            self.video_label.setText("未启动摄像头")
            self.append_output("停止测试摄像头")
    
    def update_video_frame(self, image):
        """更新视频帧"""
        if self.camera_active:
            # 调整图像大小以适应标签
            scaled_image = image.scaled(
                self.video_label.width(),
                self.video_label.height(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation
            )
            
            # 显示图像
            self.video_label.setPixmap(QPixmap.fromImage(scaled_image))
    
    def toggle_motion_control(self, checked):
        """切换运动控制状态"""
        if checked:
            # 检查ROS核心是否运行
            if self.roscore_status_label.text() != "运行中":
                self.append_error("激活运动控制失败，ROS核心未运行")
                self.activate_control_button.setChecked(False)
                return
                
            # 检查基础节点是否已启动
            try:
                # 首先检查标记文件
                base_node_running = os.path.exists("/tmp/limo_base_node_running")
                
                # 如果标记文件不存在，再通过rostopic检查
                if not base_node_running:
                    result = subprocess.run(
                        ["rostopic", "list"], 
                        stdout=subprocess.PIPE, 
                        stderr=subprocess.PIPE, 
                        text=True,
                        timeout=2
                    )
                    
                    # 检查是否有基础节点发布的关键话题
                    key_topics = ['/limo/cmd_vel', '/tf', '/limo_status']
                    for topic in key_topics:
                        if topic in result.stdout:
                            base_node_running = True
                            break
                
                if not base_node_running:
                    self.append_error("激活运动控制失败，基础节点未启动，请先启动基础节点")
                    self.activate_control_button.setChecked(False)
                    return
                    
            except Exception as e:
                self.append_error(f"检查基础节点状态时出错: {str(e)}")
                self.activate_control_button.setChecked(False)
                return
            
            # 启动运动控制
            self.append_output("正在激活运动控制...")
            if self.motion_controller.start():
                self.append_output("运动控制已激活")
                self.control_active = True
                self.motion_control_active = True
                self.activate_control_button.setText("停止控制")
                self.statusBar().showMessage("运动控制已激活")
                
                # 测试发布一个零速度命令，确认发布者工作正常
                try:
                    self.motion_controller.stop_motion()
                    self.append_output("已发送测试命令，确认发布者工作正常")
                except Exception as e:
                    self.append_error(f"发送测试命令时出错: {str(e)}")
            else:
                self.append_error("激活运动控制失败")
                self.activate_control_button.setChecked(False)
        else:
            # 停止运动控制
            self.append_output("正在停止运动控制...")
            self.motion_controller.stop()
            self.control_active = False
            self.motion_control_active = False
            self.activate_control_button.setText("激活控制")
            self.append_output("运动控制已停止")
            self.statusBar().showMessage("运动控制已停止")
            
            # 重置控制区域
          
    
    def on_up_button_pressed(self):
        """向上按钮按下事件"""
        if self.motion_control_active:
            self.motion_controller.move_forward()
    
    def on_down_button_pressed(self):
        """向下按钮按下事件"""
        if self.motion_control_active:
            self.motion_controller.move_backward()
    
    def on_left_button_pressed(self):
        """向左按钮按下事件"""
        if self.motion_control_active:
            self.motion_controller.turn_left()
    
    def on_right_button_pressed(self):
        """向右按钮按下事件"""
        if self.motion_control_active:
            self.motion_controller.turn_right()
    
    def on_direction_button_released(self):
        """方向按钮释放事件"""
        if self.motion_control_active:
            self.motion_controller.stop_motion()
    
    def on_stop_button_clicked(self):
        """停止按钮点击事件"""
        if self.motion_control_active:
            self.motion_controller.stop_motion()
    
    def on_mouse_position_changed(self, x_ratio, y_ratio):
        """鼠标位置改变事件"""
        if self.motion_control_active:
            self.motion_controller.set_from_joystick(x_ratio, y_ratio)
    
    def start_module(self, module):
        """启动指定的功能模块"""
        module_id = module["module_id"]
        module_name = module["name"]
        
        # 获取选项参数
        params = []
        if module["options"]:
            # 获取当前标签页中的模块按钮
            current_tab_index = self.tab_widget.currentIndex()
            current_category = self.categories[current_tab_index]
            
            # 在当前分类中查找模块索引
            modules_in_category = self.modules_by_category[current_category]
            for i, m in enumerate(modules_in_category):
                if m["module_id"] == module_id:
                    # 获取模块按钮
                    button_index = i // 2  # 行索引
                    col_index = i % 2      # 列索引
                    
                    # 获取当前标签页内容
                    tab_content = self.tab_widget.currentWidget()
                    scroll_area = tab_content.findChild(QScrollArea)
                    scroll_content = scroll_area.widget()
                    
                    # 获取网格布局
                    grid_layout = scroll_content.layout()
                    
                    # 获取模块按钮
                    module_button = grid_layout.itemAtPosition(button_index, col_index).widget()
                    
                    # 获取选项
                    options = module_button.get_options()
                    params = list(options.values())
                    break
        
        # 启动模块
        self.script_executor.execute_script(module_id, "start", *params)
        
        # 如果是摄像头相关模块，停止本地摄像头测试
        if "摄像头" in module_name and self.camera_active:
            self.toggle_camera_test()
        
        # 更新状态栏
        self.statusBar().showMessage(f"正在运行: {module_name}")
    
    def stop_module(self, module):
        """停止指定的功能模块"""
        module_id = module["module_id"]
        module_name = module["name"]
        
        # 停止模块
        self.script_executor.execute_script(module_id, "stop")
        
        # 更新状态栏
        self.statusBar().showMessage(f"已停止: {module_name}")
        


    
    def update_status_display(self):
        """更新状态显示"""
        # 更新状态标签
        self.status_labels["battery_voltage"].setText(f"{self.status_monitor.battery_voltage}")
        
        # 特别处理运动模式显示，确保麦轮模式能被正确显示
        # motion_mode = self.status_monitor.motion_mode
        # motion_mode_text = self.status_monitor.motion_mode_dict.get(motion_mode, "未知")
        # 如果是麦轮模式，添加特殊标记
        # if motion_mode == 2:
        #     motion_mode_text = f"{motion_mode_text} ✓"
        # self.status_labels["motion_mode"].setText(motion_mode_text)
        
        self.status_labels["linear_velocity"].setText(f"{self.status_monitor.linear_velocity:.1f}")
        self.status_labels["angular_velocity"].setText(f"{self.status_monitor.angular_velocity:.2f}")
        self.status_labels["lateral_velocity"].setText(f"{self.status_monitor.lateral_velocity:.2f}")
        self.status_labels["steering_angle"].setText(f"{self.status_monitor.steering_angle:.2f}")
        
        # 错误代码处理
        error_text = "无"
        if self.status_monitor.error_code > 0:
            error_codes = []
            for code, desc in self.status_monitor.error_code_dict.items():
                if self.status_monitor.error_code & code:
                    error_codes.append(desc)
            if error_codes:
                error_text = ", ".join(error_codes)
        self.status_labels["error_code"].setText(error_text)
    
    def update_charts(self):
        """更新图表"""
        # 获取数据锁
        with self.status_monitor.data_lock:
            timestamps = self.status_monitor.timestamps.copy()
            battery_history = self.status_monitor.battery_history.copy()
            velocity_history = self.status_monitor.velocity_history.copy()
        
        # 如果没有数据，则不更新
        if not timestamps:
            return
        
        # 转换时间戳为相对时间（秒）
        relative_times = [t - timestamps[0] for t in timestamps]
        
        # 更新电池电压图表
        self.battery_canvas.axes.clear()
        self.battery_canvas.axes.plot(relative_times, battery_history, 'b-')
        self.battery_canvas.axes.set_ylabel('电压 (V)')
        self.battery_canvas.axes.set_xlabel('时间 (s)')
        self.battery_canvas.axes.grid(True)
        # 设置Y轴刻度为整数
        import matplotlib.ticker as ticker
        self.battery_canvas.axes.yaxis.set_major_formatter(ticker.FormatStrFormatter('%d'))
        # 确保Y轴标签完全可见
        if battery_history:
            min_val = int(min(battery_history)) -1 
            max_val = int(max(battery_history)) + 1  # 加1确保上限大于最大值
            self.battery_canvas.axes.set_ylim(min_val, max_val)
        self.battery_canvas.axes.tick_params(axis='y', pad=8)
        self.battery_canvas.draw()
        
        # 更新速度图表
        self.velocity_canvas.axes.clear()
        self.velocity_canvas.axes.plot(relative_times, velocity_history, 'g-')
        self.velocity_canvas.axes.set_ylabel('线速度 (m/s)')
        self.velocity_canvas.axes.set_xlabel('时间 (s)')
        self.velocity_canvas.axes.grid(True)
        # 设置Y轴刻度为一位小数
        import matplotlib.ticker as ticker
        self.velocity_canvas.axes.yaxis.set_major_formatter(ticker.FormatStrFormatter('%.1f'))
        # 确保Y轴标签完全可见
        self.velocity_canvas.axes.tick_params(axis='y', pad=8)
        self.velocity_canvas.draw()
    
    def check_roscore_status(self):
        """检查roscore状态"""
        try:
            # 使用多种方法检查roscore是否运行
            ros_running = False
            
            # 方法1: 使用rostopic list命令
            try:
                result = subprocess.run(
                    ["rostopic", "list"], 
                    stdout=subprocess.PIPE, 
                    stderr=subprocess.PIPE, 
                    timeout=1
                )
                if result.returncode == 0:
                    ros_running = True
            except Exception as e:
                print(f"使用rostopic检查ROS状态时出现问题: {str(e)}")
            
            # 方法2: 检查ROS_MASTER_URI环境变量并尝试连接
            if not ros_running and 'ROS_MASTER_URI' in os.environ:
                try:
                    import xmlrpc.client
                    master_uri = os.environ['ROS_MASTER_URI']
                    master = xmlrpc.client.ServerProxy(master_uri)
                    # 尝试调用一个简单的API
                    master.getSystemState('/rostopic')
                    ros_running = True
                except Exception as e:
                    print("已检查ros运行状态")
            
            # 方法3: 检查roscore进程
            if not ros_running:
                try:
                    ps_result = subprocess.run(
                        ["ps", "aux"], 
                        stdout=subprocess.PIPE, 
                        stderr=subprocess.PIPE, 
                        text=True
                    )
                    if "roscore" in ps_result.stdout or "rosmaster" in ps_result.stdout:
                        ros_running = True
                except Exception as e:
                    self.append_output(f"检查roscore进程时出现问题: {str(e)}")
            
            # 根据检测结果更新状态
            if ros_running:
                # roscore正在运行
                self.roscore_status_label.setText("运行中")
                self.roscore_status_label.setStyleSheet("color: green; font-weight: bold;")
                
                # 启动ROS话题订阅
                self.status_monitor.stop_pylimo()
                self.status_monitor.start_ros_subscriber()
                
                # 尝试订阅/cmd_vel话题获取运动数据
                try:
                    if hasattr(self.status_monitor, 'ros_subscribers') and not any(getattr(sub, 'name', '') == '/cmd_vel' for sub in self.status_monitor.ros_subscribers):
                        rospy.Subscriber('/cmd_vel', Twist, self.status_monitor._velocity_callback)
                        self.append_output("已订阅/cmd_vel话题")
                except Exception as e:
                    self.append_error(f"订阅/cmd_vel话题时出现问题: {str(e)}")
            else:
                # roscore未运行
                self.roscore_status_label.setText("未运行")
                self.roscore_status_label.setStyleSheet("color: red; font-weight: bold;")
                
                # 启动pylimo库数据读取
                self.status_monitor.stop_ros_subscriber()
                self.status_monitor.start_pylimo()
        except Exception as e:
            # 发生异常，记录错误并认为roscore未运行
            self.append_error(f"检查ROS核心状态时出现问题: {str(e)}")
            self.roscore_status_label.setText("未运行")
            self.roscore_status_label.setStyleSheet("color: red; font-weight: bold;")
            
            # 启动pylimo库数据读取
            self.status_monitor.stop_ros_subscriber()
            self.status_monitor.start_pylimo()
    
    def switch_to_ros_topics(self):
        """切换到ROS话题订阅模式"""
        self.append_output("检测到ROS核心运行中，切换到ROS话题订阅模式")
        
        # 停止pylimo库数据读取
        self.status_monitor.stop_pylimo()
        
        # 启动ROS话题订阅
        self.status_monitor.start_ros_subscriber()
    
    def switch_to_pylimo(self):
        """切换到pylimo库模式"""
        self.append_output("未检测到ROS核心，切换到pylimo库读取模式")
        
        # 停止ROS话题订阅
        self.status_monitor.stop_ros_subscriber()
        
        # 启动pylimo库数据读取
        self.status_monitor.start_pylimo()
    
    def append_output(self, text):
        """添加输出文本"""
        cursor = self.output_text.textCursor()
        format = QTextCharFormat()
        format.setForeground(QColor("#54a0ff"))  # 蓝色
        cursor.movePosition(QTextCursor.End)
        cursor.insertText(text + "\n", format)
        self.output_text.setTextCursor(cursor)
        self.output_text.ensureCursorVisible()
    
    def append_error(self, text):
        """添加错误文本"""
        cursor = self.output_text.textCursor()
        format = QTextCharFormat()
        format.setForeground(QColor("#f1c40f"))  # 黄色
        cursor.movePosition(QTextCursor.End)
        cursor.insertText("警告: " + text + "\n", format)
        self.output_text.setTextCursor(cursor)
        self.output_text.ensureCursorVisible()
    
    def clear_output(self):
        """清空输出区域"""
        self.output_text.clear()
    
    def reset_all(self):
        """复位所有功能"""
        # 终止所有进程
        self.script_executor.terminate_all()
        
        # 清空输出
        self.clear_output()
        
        # 停止摄像头测试
        if self.camera_active:
            self.toggle_camera_test()
        
        # 停止运动控制
        if self.motion_control_active:
            self.activate_control_button.setChecked(False)
            self.toggle_motion_control(False)
        
        # 清除所有正在运行的任务和命令
        try:
            self.append_output("正在清除所有正在运行的任务和命令...")
            
            # 首先尝试优雅地关闭limo_base节点
            try:
                # 查找limo_base相关进程
                self.append_output("正在停止limo_base进程...")
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
                        # 先尝试使用SIGINT信号终止进程
                        subprocess.run(["kill", "-2", pid], check=False)
                        self.append_output(f"已发送SIGINT信号到进程 {pid}")
                    except Exception as e:
                        self.append_output(f"发送SIGINT信号到进程 {pid} 时出错: {str(e)}")
                
                # 等待一段时间，让进程有机会优雅地退出
                time.sleep(2)
                
                # 再次检查进程是否仍在运行
                cmd = "ps aux | grep -E 'limo_base|limo_start' | grep -v grep"
                output = subprocess.check_output(cmd, shell=True).decode('utf-8')
                
                # 如果进程仍在运行，使用SIGKILL强制终止
                if output.strip():
                    self.append_output("一些进程仍在运行，尝试强制终止...")
                    for line in output.strip().split('\n'):
                        if not line:
                            continue
                        
                        parts = line.split()
                        if len(parts) < 2:
                            continue
                            
                        pid = parts[1]
                        try:
                            subprocess.run(["kill", "-9", pid], check=False)
                            self.append_output(f"已强制终止进程 {pid}")
                        except Exception as e:
                            self.append_output(f"强制终止进程 {pid} 时出错: {str(e)}")
            except Exception as e:
                self.append_output(f"停止limo_base进程时出错: {str(e)}")
            
            # 查找并终止所有ROS相关进程
            try:
                self.append_output("正在停止所有ROS相关进程...")
                
                # 定义特殊进程列表，这些进程可能需要特殊处理
                special_processes = [
                    "roscore", "rosmaster", "rosout", "detect_node.py", "vision", 
                    "text_recognition", "voice_control"
                ]
                
                # 查找ROS相关进程
                cmd = "ps aux | grep -E 'ros|rviz|gmapping|cartographer|navigation|slam|map_server' | grep -v grep"
                output = subprocess.check_output(cmd, shell=True).decode('utf-8')
                
                # 提取PID并终止进程
                for line in output.strip().split('\n'):
                    if not line:
                        continue
                    
                    parts = line.split()
                    if len(parts) < 2:
                        continue
                    
                    pid = parts[1]
                    cmd_line = ' '.join(parts[10:]) if len(parts) > 10 else ''
                    
                    # 检查是否为特殊进程
                    is_special = any(sp in cmd_line for sp in special_processes)
                    
                    try:
                        # 先尝试使用SIGINT信号终止进程
                        subprocess.run(["kill", "-2", pid], check=False)
                        self.append_output(f"已发送SIGINT信号到进程 {pid}" + (" (特殊进程)" if is_special else ""))
                    except Exception as e:
                        self.append_output(f"发送SIGINT信号到进程 {pid} 时出错: {str(e)}")
                
                # 等待一段时间，让进程有机会优雅地退出
                time.sleep(2)
                
                # 再次检查进程是否仍在运行
                cmd = "ps aux | grep -E 'ros|rviz|gmapping|cartographer|navigation|slam|map_server' | grep -v grep"
                output = subprocess.check_output(cmd, shell=True).decode('utf-8')
                
                # 如果进程仍在运行，使用SIGKILL强制终止
                if output.strip():
                    self.append_output("一些进程仍在运行，尝试强制终止...")
                    for line in output.strip().split('\n'):
                        if not line:
                            continue
                        
                        parts = line.split()
                        if len(parts) < 2:
                            continue
                        
                        pid = parts[1]
                        try:
                            subprocess.run(["kill", "-9", pid], check=False)
                            self.append_output(f"已强制终止进程 {pid}")
                        except Exception as e:
                            self.append_output(f"强制终止进程 {pid} 时出错: {str(e)}")
                
                # 特别检查特殊进程
                cmd = "ps aux | grep -E '" + '|'.join(special_processes) + "' | grep -v grep"
                output = subprocess.check_output(cmd, shell=True).decode('utf-8')
                
                # 如果特殊进程仍在运行，使用SIGKILL强制终止
                if output.strip():
                    self.append_output("一些特殊进程仍在运行，尝试强制终止...")
                    for line in output.strip().split('\n'):
                        if not line:
                            continue
                        
                        parts = line.split()
                        if len(parts) < 2:
                            continue
                        
                        pid = parts[1]
                        try:
                            subprocess.run(["kill", "-9", pid], check=False)
                            self.append_output(f"已强制终止特殊进程 {pid}")
                        except Exception as e:
                            self.append_output(f"强制终止特殊进程 {pid} 时出错: {str(e)}")
            except Exception as e:
                self.append_output(f"停止ROS相关进程时出错: {str(e)}")
            
            # 释放串口资源
            try:
                self.append_output("正在释放串口资源...")
                # 查找使用串口的进程
                cmd = "lsof /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || true"
                output = subprocess.check_output(cmd, shell=True).decode('utf-8')
                
                # 提取PID并终止进程
                for line in output.strip().split('\n'):
                    if not line or "COMMAND" in line:  # 跳过标题行
                        continue
                    
                    parts = line.split()
                    if len(parts) < 2:
                        continue
                    
                    pid = parts[1]
                    try:
                        subprocess.run(["kill", "-9", pid], check=False)
                        self.append_output(f"已终止使用串口的进程 {pid}")
                    except Exception as e:
                        self.append_output(f"终止使用串口的进程 {pid} 时出错: {str(e)}")
            except Exception as e:
                self.append_output(f"释放串口资源时出错: {str(e)}")
            
            # 重新初始化状态监控器
            try:
                self.append_output("正在重新初始化状态监控器...")
                self.status_monitor.reinitialize()
                self.append_output("状态监控器已重新初始化")
            except Exception as e:
                self.append_output(f"重新初始化状态监控器时出错: {str(e)}")
            
            self.append_output("所有任务和命令已清除")
        except Exception as e:
            self.append_output(f"复位过程中出错: {str(e)}")
        
        # 更新状态显示
        self.update_status_display()
                  

    # 重写调整大小事件，确保窗口自适应大小
    def resizeEvent(self, event):
        super().resizeEvent(event)
        # 可以在这里添加额外的自适应大小逻辑
        # 例如，根据窗口大小调整某些组件的大小

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = LimoConsole()
    window.show()
    sys.exit(app.exec_())
