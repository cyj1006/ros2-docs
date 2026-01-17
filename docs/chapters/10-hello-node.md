# 第十章：ROS2 核心概念详解——最重要的知识点

这一章讲 ROS2 的几个 **核心概念**。

!!! success "重要性"
    理解了这些，后面学起来会顺很多。
    
    这是整个 ROS2 的基础。

---

## 10.1 节点（Node）

### 什么是节点

!!! info "定义"
    节点就是一个 **独立运行的程序**。

一个机器人系统通常由 **多个节点** 组成，每个节点负责一件事。

---

### 举例说明

比如一个节点负责：

- 读摄像头
- 处理图像
- 控制电机

它们各自独立运行，通过 **消息通信** 来协作。

---

### 为什么要这样设计

!!! success "模块化的好处"
    - ✓ **改一个功能只需要改对应的节点**
      - 其他不用动
    
    - ✓ **某个节点出问题了**
      - 其他节点还能继续跑
    
    - ✓ **调试方便**
      - 可以单独测试每个节点

---

### 节点的常用命令

```bash
# 列出当前运行的节点
ros2 node list

# 查看节点的详细信息
ros2 node info /节点名
```

---

#### 实例

```bash
# 启动小海龟
ros2 run turtlesim turtlesim_node

# 另一个终端查看节点
ros2 node list
# 输出：/turtlesim

# 查看小海龟节点的详细信息
ros2 node info /turtlesim
```

!!! info "node info 显示什么"
    可以看到节点的：
    - 订阅了哪些话题
    - 发布了哪些话题
    - 提供了哪些服务
    - 使用了哪些参数

---

## 10.2 话题（Topic）

### 什么是话题

!!! info "定义"
    话题是节点之间 **传递数据的通道**。

采用 **"发布-订阅"** 模式：

- **发布者（Publisher）** 往话题上发数据
- **订阅者（Subscriber）** 从话题上收数据

---

### 特点

!!! info "解耦设计"
    - 发布者不管谁在收
    - 订阅者也不管谁在发
    - 它们之间没有直接联系

---

### 一对多关系

!!! success "灵活性"
    一个话题可以有：
    - **多个发布者**
    - **多个订阅者**

---

### 举例

!!! example "实际场景"
    摄像头节点往 `/camera/image` 话题发图像：
    
    - 图像处理节点订阅这个话题来处理
    - 同时显示节点也订阅这个话题来显示给你看

---

### 常用命令

```bash
# 列出所有话题
ros2 topic list

# 实时查看话题上的数据
ros2 topic echo /话题名

# 查看话题发布频率
ros2 topic hz /话题名

# 查看话题信息
ros2 topic info /话题名

# 手动发布消息
ros2 topic pub /话题名 类型 "数据"
```

---

#### 实例

```bash
# 启动小海龟
ros2 run turtlesim turtlesim_node

# 查看所有话题
ros2 topic list
# 输出：
# /turtle1/cmd_vel
# /turtle1/pose
# /turtle1/color_sensor
# ...

# 实时查看小海龟的位置
ros2 topic echo /turtle1/pose

# 查看速度话题的发布频率
ros2 topic hz /turtle1/cmd_vel

# 手动发布速度指令（让小海龟动起来）
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}"
```

!!! tip "调试神器"
    `ros2 topic echo` 是调试时 **最常用的命令**，
    
    可以实时看到数据流动，非常直观。

---

## 10.3 消息（Message）

### 什么是消息

!!! info "定义"
    消息是话题上传输的 **数据的格式**。

每个话题有固定的消息类型，

就像快递包裹有不同规格一样。

---

### 常用的消息类型

#### 1. std_msgs/String

```
string data
```

!!! info "用途"
    传输字符串

---

#### 2. geometry_msgs/Twist

```
Vector3 linear
    float64 x
    float64 y
    float64 z
Vector3 angular
    float64 x
    float64 y
    float64 z
```

!!! info "用途"
    控制机器人移动
    - linear：线速度
    - angular：角速度

---

#### 3. sensor_msgs/Image

!!! info "用途"
    传输图像数据

---

#### 4. sensor_msgs/LaserScan

!!! info "用途"
    激光雷达数据

---

### 查看消息结构

```bash
# 查看消息的详细结构
ros2 interface show geometry_msgs/msg/Twist

# 列出所有消息类型
ros2 interface list
```

---

#### 实例

```bash
# 查看 Twist 消息的结构
ros2 interface show geometry_msgs/msg/Twist

# 输出：
# Vector3 linear
#     float64 x
#     float64 y
#     float64 z
# Vector3 angular
#     float64 x
#     float64 y
#     float64 z
```

!!! tip "编程时的作用"
    了解消息结构，方便编程时使用。

---

## 10.4 服务（Service）

### 什么是服务

!!! info "定义"
    服务是 **"请求-响应"** 模式的通信。

---

### 工作流程

1. **客户端** 发一个请求
2. **服务端** 处理后返回响应

---

### 与话题的区别

| 特性 | 话题 | 服务 |
|------|------|------|
| **通信模式** | 单向，持续数据流 | 双向，一问一答 |
| **是否等待** | 不等待响应 | 等待响应 |
| **适用场景** | 传感器数据 | 获取状态、执行操作 |

---

### 适用场景

!!! example "实际应用"
    - 获取某个状态
    - 执行某个操作并等待结果

比如：

- 问问机器人现在电量多少
- 让机器人重置位置

---

### 常用命令

```bash
# 列出所有服务
ros2 service list

# 调用服务
ros2 service call /服务名 类型 "请求数据"

# 查看服务类型
ros2 service type /服务名
```

---

#### 实例

```bash
# 启动小海龟
ros2 run turtlesim turtlesim_node

# 查看所有服务
ros2 service list
# 输出：
# /clear
# /reset
# /spawn
# ...

# 调用清除轨迹的服务
ros2 service call /clear std_srvs/srv/Empty

# 重置小海龟位置
ros2 service call /reset std_srvs/srv/Empty

# 生成新的海龟
ros2 service call /spawn turtlesim/srv/Spawn \
    "{x: 2.0, y: 2.0, theta: 0.0, name: 'turtle2'}"
```

---

## 10.5 动作（Action）

### 什么是动作

!!! info "定义"
    动作用于 **需要较长时间的任务**。

---

### 工作流程

1. 你发送一个 **目标**
2. 执行过程中可以收到 **进度反馈**
3. 最后收到 **结果**
4. 中途还能 **取消**

---

### 类比理解

!!! example "像网购"
    - 你下单（发送目标）
    - 然后物流一直告诉你到哪了（反馈）
    - 最后收到货（结果）
    - 中间你还能取消订单

---

### 适用场景

!!! example "实际应用"
    - 导航到某个地点
    - 机械臂执行某个动作
    - 任何需要较长时间的任务

---

### 动作的组成

一个动作包含三部分：

1. **Goal（目标）**
   - 你想让机器人做什么

2. **Feedback（反馈）**
   - 执行过程中的进度

3. **Result（结果）**
   - 最终的执行结果

---

### 常用命令

```bash
# 列出所有动作
ros2 action list

# 发送目标
ros2 action send_goal /动作名 类型 "目标数据"
```

---

#### 实例

```bash
# 控制小海龟转圈
ros2 action send_goal /turtle1/rotate_absolute \
    turtlesim/action/RotateAbsolute "{theta: 1.57}" \
    --feedback
```

---

## 10.6 参数（Parameter）

### 什么是参数

!!! info "定义"
    参数是节点的 **配置值**，
    
    可以在 **运行时修改**。

---

### 举例

!!! example "实际应用"
    - 机器人的最大速度
    - 传感器的采样频率
    - 背景颜色

这些都可以做成参数，

不用改代码就能调整。

---

### 常用命令

```bash
# 列出节点的所有参数
ros2 param list

# 获取参数值
ros2 param get /节点名 参数名

# 设置参数值
ros2 param set /节点名 参数名 值

# 保存参数到文件
ros2 param dump /节点名

# 从文件加载参数
ros2 param load /节点名 参数文件
```

---

#### 实例

```bash
# 启动小海龟
ros2 run turtlesim turtlesim_node

# 查看小海龟节点的参数
ros2 param list /turtlesim
# 输出：
# background_b
# background_g
# background_r
# ...

# 查看背景红色值
ros2 param get /turtlesim background_r

# 修改背景颜色（改成红色）
ros2 param set /turtlesim background_r 255
ros2 param set /turtlesim background_g 0
ros2 param set /turtlesim background_b 0

# 需要调用 clear 服务才能看到效果
ros2 service call /clear std_srvs/srv/Empty
```

---

## 10.7 包（Package）

### 什么是包

!!! info "定义"
    包是代码的 **组织单位**。

你写的每个功能模块都放在一个包里。

---

### 包里面有什么

- 代码文件
- 配置文件
- 启动文件
- 资源文件

---

### 创建包的命令

```bash
# Python 包
ros2 pkg create --build-type ament_python 包名 \
    --dependencies rclpy std_msgs

# C++ 包
ros2 pkg create --build-type ament_cmake 包名 \
    --dependencies rclcpp std_msgs
```

---

#### 实例

```bash
cd ~/ros2_ws/src

# 创建一个 Python 包
ros2 pkg create --build-type ament_python my_robot_pkg \
    --dependencies rclpy std_msgs geometry_msgs
```

---

### 包的常用命令

```bash
# 列出所有包
ros2 pkg list

# 查找包的路径
ros2 pkg prefix 包名

# 查看包的可执行文件
ros2 pkg executables 包名
```

---

## 10.8 启动文件（Launch File）

### 什么是启动文件

!!! info "定义"
    启动文件用来 **一次启动多个节点**，
    
    还可以配置参数。

---

### 为什么需要

!!! warning "没有启动文件的痛苦"
    如果你的系统有 10 个节点，
    
    没有启动文件的话，你要开 10 个终端，
    
    分别运行 10 次 `ros2 run`。

!!! success "有启动文件"
    一行命令全部启动。

---

### 启动文件的类型

ROS2 的启动文件是 **Python 脚本**，

文件名通常是 `xxx.launch.py`。

---

### 运行启动文件

```bash
ros2 launch 包名 xxx.launch.py
```

---

#### 实例

```bash
# 启动 navigation2 导航
ros2 launch nav2_bringup navigation_launch.py
```

---

### 简单的启动文件示例

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            executable='turtle_teleop_key',
            name='teleop',
            output='screen'
        ),
    ])
```

!!! info "这个启动文件会"
    同时启动小海龟和键盘控制节点。

---

## 10.9 工作空间（Workspace）

### 什么是工作空间

!!! info "定义"
    工作空间是放你 **自己开发的包** 的地方。

---

### 工作空间的结构

```
ros2_ws/
├── src/           # 源代码
│   ├── package1/
│   ├── package2/
│   └── ...
├── build/         # 编译产物
├── install/       # 安装文件
└── log/           # 日志
```

---

### 创建工作空间

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
```

---

### 使用工作空间

```bash
# 编译工作空间
cd ~/ros2_ws
colcon build

# 加载工作空间环境
source install/setup.bash
```

---

## 10.10 坐标变换（TF2）

### 什么是 TF2

!!! info "定义"
    TF2 用来管理 **坐标系之间的关系**。

---

### 为什么需要

机器人身上有很多传感器：

- 摄像头
- 激光雷达
- IMU

它们各自有自己的坐标系。

!!! warning "问题"
    摄像头看到一个物体在它前方 1 米，
    
    但这 1 米在机器人坐标系里是哪里？

---

### TF2 的作用

!!! success "解决方案"
    TF2 自动处理这些坐标变换，
    
    你可以查询任意两个坐标系之间的关系。

---

### 查看 TF 树

```bash
# 查看当前的 TF 树
ros2 run tf2_tools view_frames

# 会生成一个 PDF 文件显示坐标系关系
```

---

### 可视化 TF

在 RViz2 里可以显示 TF，

很直观地看到各个坐标系的位置关系。

---

## 10.11 核心概念关系图

```
机器人系统
    ↓
由多个节点组成
    ↓
节点之间通过三种方式通信：
    ├── 话题（Topic）        → 持续数据流
    ├── 服务（Service）      → 一问一答
    └── 动作（Action）       → 长时任务
        ↓
传输的数据有固定格式（消息 Message）
    ↓
节点有配置项（参数 Parameter）
    ↓
节点组织在包（Package）里
    ↓
包放在工作空间（Workspace）里
    ↓
可以用启动文件（Launch）一次启动多个节点
    ↓
坐标系用 TF2 管理
```

---

## 10.12 小海龟系统分析

### 以小海龟为例，分析 ROS2 系统

#### 节点

```
/turtlesim        # 小海龟仿真节点
/teleop_turtle    # 键盘控制节点
```

---

#### 话题

```
/turtle1/cmd_vel  # 速度指令（teleop → turtlesim）
/turtle1/pose     # 位置信息（turtlesim → 外界）
```

---

#### 服务

```
/clear            # 清除轨迹
/reset            # 重置
/spawn            # 生成新海龟
```

---

#### 参数

```
backgroun
background_g      # 背景绿色值
background_b      # 背景蓝色值
```

---

### 数据流动

```
键盘输入
    ↓
teleop_turtle 节点
    ↓
发布到 /turtle1/cmd_vel 话题
    ↓
turtlesim 节点订阅
    ↓
控制小海龟移动
    ↓
发布位置到 /turtle1/pose 话题
```

---

## 本章小结

通过这一章，你应该已经掌握：

1. **节点（Node）**

    独立运行的程序单元

2. **话题（Topic）**

    发布-订阅模式
   
    用于持续数据流

3. **服务（Service）**

    请求-响应模式

    用于一问一答

4. **动作（Action）**

    用于长时任务

    有反馈和结果

5. **消息（Message）**

    数据的格式定义

6. **参数（Parameter）**

    节点的配置项

7. **包（Package）**

    代码组织单位

8. **启动文件（Launch）**

    批量启动节点

9. **工作空间（Workspace）**

    开发环境

10. **TF2**

     坐标变换管理

---

## 下一章预告

下一章我们会 **开始写代码**：

> **第一个 ROS2 程序——Hello World**

手把手教你创建第一个 ROS2 节点，

从实践中加深对概念的理解。

