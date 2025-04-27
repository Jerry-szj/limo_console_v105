# LIMO小车控制台说明文档

## 1. 概述

LIMO小车控制台是一个用于控制和监控LIMO机器人的图形用户界面应用程序。该控制台集成了多种功能，包括状态监控、功能模块控制、视频显示和运动控制等。本文档详细介绍了控制台的代码结构、主要组件和功能说明，以便后续开发人员能够理解和维护这个系统。

## 2. 系统架构

LIMO小车控制台采用模块化设计，主要由以下几个核心组件组成：

1. **主控制台界面**（new_limo_console.py）：提供图形用户界面，集成所有功能模块
2. **状态监控器**（status_monitor.py）：负责监控LIMO底盘状态，支持pylimo库和ROS话题两种数据源
3. **脚本执行器**（script_executor.py）：负责执行各种功能脚本
4. **摄像头管理器**（camera_manager.py）：负责管理摄像头视频流
5. **运动控制器**（motion_controller.py）：负责控制LIMO的运动
6. **数据模拟器**（data_simulator.py）：在无法获取真实数据时提供模拟数据
7. **功能脚本**（scripts目录）：包含各种功能模块的实现脚本

系统架构图如下：

```
+------------------------+
|   主控制台界面         |
|  (new_limo_console.py) |
+------------------------+
          |
          v
+------------------------+     +------------------------+
|      状态监控器        |     |      脚本执行器       |
|  (status_monitor.py)   |     |  (script_executor.py) |
+------------------------+     +------------------------+
          |                              |
          v                              v
+------------------------+     +------------------------+
|     运动控制器         |     |     功能脚本          |
| (motion_controller.py) |     |   (scripts/*.py)      |
+------------------------+     +------------------------+
          |                              |
          v                              v
+------------------------+     +------------------------+
|    摄像头管理器        |     |     数据模拟器        |
|  (camera_manager.py)   |     |  (data_simulator.py)  |
+------------------------+     +------------------------+
```

## 3. 主要组件详细说明

### 3.1 主控制台界面 (new_limo_console.py)

主控制台界面是整个系统的核心，它集成了所有功能模块，并提供了用户交互界面。

#### 主要类：

- **LimoConsole**：主窗口类，继承自QMainWindow
- **MatplotlibCanvas**：Matplotlib画布类，用于在Qt界面中嵌入matplotlib图表
- **ModuleButton**：模块按钮类，用于创建统一样式的功能模块按钮
- **MouseControlArea**：鼠标控制区域类，用于捕获鼠标事件并控制速度

#### 主要功能：

1. **界面布局**：分为左侧功能面板和右侧状态/命令面板
2. **功能模块管理**：按分类组织各种功能模块
3. **状态显示**：显示LIMO的各种状态信息，包括电池电压、速度等
4. **命令输出**：显示命令执行的输出信息
5. **地图保存**：提供一键保存地图功能
6. **ROS核心管理**：提供启动和停止ROS核心的功能

### 3.2 状态监控器 (status_monitor.py)

状态监控器负责监控LIMO底盘状态，支持pylimo库和ROS话题两种数据源。

#### 主要类：

- **StatusMonitor**：状态监控类，继承自QObject

#### 主要功能：

1. **数据获取**：支持从pylimo库、ROS话题和模拟数据三种来源获取数据
2. **状态更新**：实时更新LIMO的各种状态信息
3. **历史数据记录**：记录电池电压和速度的历史数据，用于绘制图表
4. **错误处理**：处理数据获取过程中的各种错误情况

### 3.3 脚本执行器 (script_executor.py)

脚本执行器负责执行各种功能脚本，并处理脚本执行过程中的输出和错误信息。

#### 主要类：

- **ScriptExecutor**：脚本执行类，继承自QObject

#### 主要功能：

1. **脚本执行**：执行指定的Python脚本
2. **输出处理**：处理脚本执行过程中的标准输出和错误输出
3. **进程管理**：管理脚本执行进程，包括启动、停止和监控

### 3.4 摄像头管理器 (camera_manager.py)

摄像头管理器负责管理摄像头视频流，支持本地摄像头和ROS图像话题两种数据源。

#### 主要类：

- **CameraManager**：摄像头管理类，继承自QObject

#### 主要功能：

1. **视频获取**：从本地摄像头或ROS图像话题获取视频流
2. **图像处理**：处理获取到的图像，包括格式转换、大小调整等
3. **错误处理**：处理视频获取过程中的各种错误情况

### 3.5 运动控制器 (motion_controller.py)

运动控制器负责控制LIMO的运动，支持直接控制和ROS话题发布两种方式。

#### 主要类：

- **MotionController**：运动控制类

#### 主要功能：

1. **速度控制**：控制LIMO的线速度和角速度
2. **运动模式切换**：切换LIMO的运动模式，包括差速模式、阿克曼模式和麦轮模式
3. **ROS话题发布**：通过ROS话题发布速度命令

### 3.6 数据模拟器 (data_simulator.py)

数据模拟器在无法获取真实数据时提供模拟数据，用于测试和演示。

#### 主要类：

- **DataSimulator**：数据模拟类

#### 主要功能：

1. **数据生成**：生成模拟的状态数据，包括电池电压、速度等
2. **数据更新**：定期更新模拟数据，模拟真实数据的变化

### 3.7 功能脚本 (scripts目录)

功能脚本目录包含各种功能模块的实现脚本，每个脚本对应一个功能模块。

#### 主要脚本：

1. **module1_gmapping.py**：Gmapping建图功能
2. **module2_cartographer.py**：Cartographer建图功能
3. **module3_check_lidar.py**：检查雷达数据功能
4. **module4_navigation.py**：雷达导航功能
5. **map_saver.py**：地图保存功能
6. 其他功能模块脚本...

## 4. 关键功能实现

### 4.1 地图保存功能

地图保存功能由`map_saver.py`脚本实现，支持Gmapping和Cartographer两种建图方式。

#### 实现流程：

1. 检测当前运行的建图类型
2. 根据建图类型调用相应的保存函数
3. 保存地图文件到指定目录
4. 返回保存结果

#### 关键代码：

```python
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
```

### 4.2 ROS数据获取功能

ROS数据获取功能由`status_monitor.py`中的`start_ros_subscriber`方法实现，通过订阅ROS话题获取LIMO的状态数据。

#### 实现流程：

1. 初始化ROS节点
2. 创建订阅者，订阅各种状态话题
3. 在回调函数中处理接收到的数据
4. 更新状态数据并发送信号

#### 关键代码：

```python
def start_ros_subscriber(self):
    """启动ROS话题订阅"""
    if self.ros_running:
        return True
    
    try:
        # 初始化ROS节点（如果尚未初始化）
        if not rospy.get_node_uri():
            try:
                rospy.init_node('limo_status_monitor', anonymous=True)
            except Exception as e:
                print(f"初始化ROS节点时出错: {str(e)}")
                # 可能是ROS主节点未运行，启动模拟数据
                self.start_simulation()
                return False
        
        # 创建订阅者
        self.ros_subscribers = [
            rospy.Subscriber('/limo/battery_voltage', Float32, self._battery_callback),
            rospy.Subscriber('/limo/motion_mode', Int32, self._motion_mode_callback),
            rospy.Subscriber('/limo/control_mode', Int32, self._control_mode_callback),
            rospy.Subscriber('/limo/velocity', Twist, self._velocity_callback),
            rospy.Subscriber('/limo/steering_angle', Float32, self._steering_angle_callback),
            rospy.Subscriber('/limo/error_code', Int32, self._error_code_callback),
            # 添加对/cmd_vel话题的订阅
            rospy.Subscriber('/cmd_vel', Twist, self._velocity_callback),
            # 添加对/limo_status话题的订阅，用于获取更全面的状态信息
            rospy.Subscriber('/limo_status', Int32, self._limo_status_callback)
        ]
        
        # 停止模拟数据
        self.stop_simulation()
        
        self.ros_running = True
        print("已启动ROS话题订阅")
        return True
    except Exception as e:
        print(f"启动ROS话题订阅时出错: {str(e)}")
        
        # 启动模拟数据生成
        self.start_simulation()
        return False
```

## 5. 修复的问题说明

在v9.1版本中，存在两个主要问题，已经进行了修复：

### 5.1 地图保存功能问题

**问题描述**：在v9.1版本中，地图保存功能不可用，会报错。

**问题原因**：在v9.1版本中，`save_map`方法内部定义了pyqtSignal信号（output_signal和error_signal），但在PyQt中，信号必须在类级别定义，而不能在方法内部定义，这导致了错误。

**修复方法**：将v9.1版本中错误的信号定义方式替换为v8.0版本中的正确实现，使用线程直接读取进程输出并更新UI。

**修复代码**：

```python
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
        
        # 创建进程
        process = subprocess.Popen(
            [sys.executable, script_path, "save"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        
        # 创建线程读取输出
        def read_output():
            while True:
                output = process.stdout.readline()
                if output:
                    self.append_output(output.strip())
                
                error = process.stderr.readline()
                if error:
                    self.append_error(error.strip())
                
                # 检查进程是否结束
                if process.poll() is not None:
                    break
        
        # 启动线程
        thread = threading.Thread(target=read_output)
        thread.daemon = True
        thread.start()
        
        # 更新状态栏
        self.statusBar().showMessage("正在保存地图...")
    
    except Exception as e:
        self.append_error(f"保存地图时出错: {str(e)}")
```

### 5.2 ROS数据获取问题

**问题描述**：在v9.1版本中，当roscore运行后获取的数据来源有问题，而v8.0版本的ros数据获取是正常的。

**问题原因**：在v9.1版本中，存在两个主要问题：
1. `_limo_status_callback`方法只是一个空的pass实现，而v8.0版本中这个方法有完整的实现，用于解析状态信息并更新运动模式
2. v9.1版本使用了动态导入和重新加载rospy模块的机制，这可能导致在roscore运行后无法正确获取数据

**修复方法**：
1. 将v8.0版本中完整的`_limo_status_callback`方法实现移植到v9.1版本
2. 移除动态导入和重新加载rospy模块的机制，改为直接导入rospy模块
3. 简化ROS订阅相关代码，确保与v8.0版本的核心功能一致

**修复代码**：

```python
# 直接导入rospy，而不是动态导入
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Int32

# 修复_limo_status_callback方法
def _limo_status_callback(self, msg):
    """LIMO状态回调函数，用于获取更全面的状态信息，包括麦轮模式"""
    with self.data_lock:
        # 检查消息中是否包含运动模式信息
        # 通常LIMO状态消息会包含运动模式信息
        # 这里我们假设msg.data的某些位表示运动模式
        # 提取运动模式 (假设运动模式在低4位)
        motion_mode = msg.data & 0x0F
        
        # 如果检测到麦轮模式 (值为2)，则更新运动模式
        if motion_mode == 2:
            self.motion_mode = motion_mode
            print(f"检测到麦轮模式: {self.motion_mode}")
    
    # 发送信号
    self.status_updated.emit()
```

## 6. 使用说明

### 6.1 启动控制台

要启动LIMO控制台，请执行以下命令：

```bash
cd /path/to/limo_console
python3 start_console.py
```

### 6.2 功能模块使用

控制台左侧面板按分类组织了各种功能模块，包括：

1. **调试类**：用于调试和测试
2. **建图类**：用于建立地图
3. **导航类**：用于导航和路径规划
4. **摄像头类**：用于摄像头相关功能
5. **雷达类**：用于雷达相关功能

每个功能模块都有"开始"和"结束"按钮，点击"开始"按钮启动相应功能，点击"结束"按钮停止功能。

### 6.3 地图保存

要保存地图，请按照以下步骤操作：

1. 确保ROS核心已启动（状态显示为"运行中"）
2. 确保已经启动了建图功能（Gmapping或Cartographer）
3. 点击"一键保存地图"按钮
4. 等待地图保存完成，保存的地图文件将位于`~/agilex_ws/src/limo_ros/limo_bringup/maps/`目录下

### 6.4 ROS核心管理

控制台提供了ROS核心管理功能，可以通过以下按钮操作：

- **启动ROS核心**：启动ROS核心
- **停止ROS核心**：停止ROS核心
- **检查ROS核心**：检查ROS核心状态

## 7. 开发指南

### 7.1 添加新功能模块

要添加新的功能模块，请按照以下步骤操作：

1. 在`scripts`目录下创建新的Python脚本，实现功能模块的逻辑
2. 在`new_limo_console.py`文件中的`_define_module_categories`方法中添加新模块的定义
3. 重新启动控制台，新模块将出现在相应的分类中

### 7.2 修改现有功能模块

要修改现有功能模块，请直接编辑`scripts`目录下的相应Python脚本。

### 7.3 调试技巧

1. 使用控制台右下方的命令输出区域查看脚本执行的输出和错误信息
2. 使用`print`语句在脚本中输出调试信息
3. 检查ROS话题和节点，确保它们正常运行

## 8. 常见问题解答

### 8.1 控制台启动失败

**问题**：控制台无法启动或启动后立即崩溃。

**解决方案**：
- 检查Python环境和依赖库是否正确安装
- 检查PyQt5是否正确安装
- 检查日志文件，查找错误信息

### 8.2 ROS核心无法启动

**问题**：点击"启动ROS核心"按钮后，ROS核心状态仍显示为"未运行"。

**解决方案**：
- 检查ROS环境是否正确设置
- 检查是否有其他ROS核心实例正在运行
- 尝试在终端中手动启动ROS核心，查看错误信息

### 8.3 地图保存失败

**问题**：点击"一键保存地图"按钮后，地图保存失败。

**解决方案**：
- 确保ROS核心已启动
- 确保已经启动了建图功能
- 检查命令输出区域中的错误信息
- 检查地图保存目录是否存在且有写入权限

### 8.4 摄像头无法显示

**问题**：摄像头视频无法显示。

**解决方案**：
- 确保摄像头已连接并被系统识别
- 检查摄像头设备路径是否正确
- 尝试使用其他工具（如`cheese`）测试摄像头

## 9. 总结

LIMO小车控制台是一个功能丰富的图形用户界面应用程序，用于控制和监控LIMO机器人。它采用模块化设计，支持多种功能模块，包括状态监控、建图、导航、摄像头控制等。本文档详细介绍了控制台的代码结构、主要组件和功能说明，以及修复的问题，希望能够帮助后续开发人员理解和维护这个系统。
