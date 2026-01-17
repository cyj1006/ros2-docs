# 第十二章：项目实战——发布者与订阅者通信

现在做个稍微复杂点的：

!!! info "项目目标"
    **一个节点发消息，另一个节点收消息。**

这是 ROS2 中 **最常用的通信方式**。

---

## 12.1 项目概述

### 要实现什么

我们要创建两个节点：

1. **发布者节点（Publisher）**
   - 不停地往话题上发消息

2. **订阅者节点（Subscriber）**
   - 从话题上接收消息并打印

---

### 通信流程

```
发布者节点
    ↓
发布消息到话题 /chatter
    ↓
订阅者节点订阅话题 /chatter
    ↓
接收并处理消息
```

---

## 12.2 创建功能包

### 创建包

```bash
cd ~/ros2_ws/src

ros2 pkg create --build-type ament_python my_pubsub \
    --dependencies rclpy std_msgs
```

!!! info "依赖说明"
    - `rclpy`：ROS2 Python 客户端库
    - `std_msgs`：标准消息类型（包含 String）

---

## 12.3 编写发布者节点

### 创建文件

```bash
cd my_pubsub/my_pubsub
touch publisher_node.py
```

---

### 发布者代码

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PublisherNode(Node):
    """发布者节点"""
    
    def __init__(self):
        super().__init__('publisher_node')
        
        # 创建发布者
        # create_publisher(消息类型, 话题名, 队列大小)
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        
        # 创建定时器，每0.5秒发布一次
        self.timer = self.create_timer(0.5, self.timer_callback)
        
        # 消息计数器
        self.count = 0
        
        self.get_logger().info('发布者节点已启动')
    
    def timer_callback(self):
        """定时器回调函数"""
        # 创建消息对象
        msg = String()
        
        # 设置消息内容
        msg.data = f'消息 #{self.count}'
        
        # 发布消息
        self.publisher_.publish(msg)
        
        # 打印日志
        self.get_logger().info(f'发布: {msg.data}')
        
        # 计数器加1
        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

### 代码详解

#### 1. 导入消息类型

```python
from std_msgs.msg import String
```

!!! info "消息类型"
    `String` 是标准消息类型，包含一个字符串字段 `data`。

---

#### 2. 创建发布者

```python
self.publisher_ = self.create_publisher(String, 'chatter', 10)
```

!!! info "参数说明"
    - `String`：消息类型
    - `'chatter'`：话题名
    - `10`：队列大小（QoS 相关）

---

#### 3. 创建定时器

```python
self.timer = self.create_timer(0.5, self.timer_callback)
```

!!! info "周期"
    每 0.5 秒调用一次 `timer_callback`

---

#### 4. 发布消息

```python
msg = String()              # 创建消息对象
msg.data = f'消息 #{self.count}'  # 设置内容
self.publisher_.publish(msg)    # 发布
```

---

## 12.4 编写订阅者节点

### 创建文件

```bash
cd ~/ros2_ws/src/my_pubsub/my_pubsub
touch subscriber_node.py
```

---

### 订阅者代码

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SubscriberNode(Node):
    """订阅者节点"""
    
    def __init__(self):
        super().__init__('subscriber_node')
        
        # 创建订阅者
        # create_subscription(消息类型, 话题名, 回调函数, 队列大小)
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10
        )
        
        self.get_logger().info('订阅者节点已启动，等待消息...')
    
    def listener_callback(self, msg):
        """接收消息的回调函数"""
        self.get_logger().info(f'收到: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = SubscriberNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

### 代码详解

#### 1. 创建订阅者

```python
self.subscription = self.create_subscription(
    String,                  # 消息类型
    'chatter',               # 话题名（要和发布者一致）
    self.listener_callback,  # 回调函数
    10                       # 队列大小
)
```

!!! warning "重要"
    话题名必须和发布者的 **完全一致**！

---

#### 2. 回调函数

```python
def listener_callback(self, msg):
    """接收消息的回调函数"""
    self.get_logger().info(f'收到: {msg.data}')
```

!!! info "工作原理"
    - 每次话题上有新消息，这个函数就会被调用
    - `msg` 参数就是接收到的消息对象

---

## 12.5 配置 setup.py

### 编辑 setup.py

```bash
cd ~/ros2_ws/src/my_pubsub
nano setup.py
```

---

### 添加入口点

找到 `entry_points`，修改为：

```python
entry_points={
    'console_scripts': [
        'publisher = my_pubsub.publisher_node:main',
        'subscriber = my_pubsub.subscriber_node:main',
    ],
},
```

!!! info "说明"
    - `publisher`：发布者的命令名
    - `subscriber`：订阅者的命令名

---

## 12.6 编译和运行

### 编译

```bash
cd ~/ros2_ws
colcon build --packages-select my_pubsub
source install/setup.bash
```

---

### 运行发布者

打开第一个终端：

```bash
ros2 run my_pubsub publisher
```

!!! success "应该看到"
    ```
    [INFO] [publisher_node]: 发布者节点已启动
    [INFO] [publisher_node]: 发布: 消息 #0
    [INFO] [publisher_node]: 发布: 消息 #1
    [INFO] [publisher_node]: 发布: 消息 #2
    ...
    ```

---

### 运行订阅者

打开第二个终端（先 source）：

```bash
source ~/ros2_ws/install/setup.bash
ros2 run my_pubsub subscriber
```

!!! success "应该看到"
    ```
    [INFO] [subscriber_node]: 订阅者节点已启动，等待消息...
    [INFO] [subscriber_node]: 收到: 消息 #5
    [INFO] [subscriber_node]: 收到: 消息 #6
    [INFO] [subscriber_node]: 收到: 消息 #7
    ...
    ```

!!! success "成功标志"
    **一个在发，一个在收！**

---

## 12.7 使用命令行工具调试

### 查看话题

打开第三个终端：

```bash
source ~/ros2_ws/install/setup.bash

# 列出所有话题
ros2 topic list
```

!!! info "应该看到"
    ```
    /chatter
    /parameter_events
    /rosout
    ```

---

### 查看话题信息

```bash
ros2 topic info /chatter
```

!!! info "会显示"
    - 发布者数量：1
    - 订阅者数量：1
    - 消息类型

---

### 实时查看消息

```bash
ros2 topic echo /chatter
```

!!! info "作用"
    可以直接看到话题上的消息，
    
    不用写订阅者节点。

---

### 查看发布频率

```bash
ros2 topic hz /chatter
```

!!! info "会显示"
    ```
    average rate: 2.000
    min: 0.500s max: 0.500s
    ```
    
    因为我们设置的是 0.5 秒发一次，所以频率是 2 Hz。

---

### 手动发布消息

```bash
ros2 topic pub /chatter std_msgs/msg/String \
    "{data: '手动发送的消息'}"
```

!!! info "作用"
    可以手动往话题上发消息，
    
    不用写发布者节点。
    
    订阅者会收到这条消息。

---

### 查看节点关系图

```bash
ros2 run rqt_graph rqt_graph
```

!!! success "可视化"
    会弹出一个图形界面，显示：
    
    - 节点之间的连接关系
    - 话题的流向

---

## 12.8 进阶：发布复杂消息

### 使用 Twist 消息

`Twist` 消息用来控制机器人移动，包含线速度和角速度。

---

### 创建新的发布者

`my_pubsub/my_pubsub/twist_publisher.py`：

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class TwistPublisher(Node):
    """发布 Twist 消息的节点"""
    
    def __init__(self):
        super().__init__('twist_publisher')
        
        # 创建发布者
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # 创建定时器
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.get_logger().info('Twist 发布者已启动')
    
    def timer_callback(self):
        """发布 Twist 消息"""
        msg = Twist()
        
        # 设置线速度（x轴向前）
        msg.linear.x = 1.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        
        # 设置角速度（z轴旋转）
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.5
        
        self.publisher_.publish(msg)
        
        self.get_logger().info(
            f'发布速度: 线速度={msg.linear.x}, 角速度={msg.angular.z}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = TwistPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

### 修改 package.xml

添加依赖：

```bash
nano ~/ros2_ws/src/my_pubsub/package.xml
```

在 `<exec_depend>` 部分添加：

```xml
<depend>geometry_msgs</depend>
```

---

### 修改 setup.py

在 `entry_points` 里添加：

```python
'twist_pub = my_pubsub.twist_publisher:main',
```

---

### 编译运行

```bash
cd ~/ros2_ws
colcon build --packages-select my_pubsub
source install/setup.bash

ros2 run my_pubsub twist_pub
```

---

### 查看消息

另一个终端：

```bash
ros2 topic echo /cmd_vel
```

!!! info "会看到"
    ```
    linear:
      x: 1.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.5
    ```

---

## 12.9 使用启动文件

### 为什么需要启动文件

!!! warning "麻烦的方式"
    每次要开两个终端，分别运行发布者和订阅者。

!!! success "启动文件"
    用启动文件可以一次启动多个节点。

---

### 创建 launch 目录

```bash
cd ~/ros2_ws/src/my_pubsub
mkdir launch
```

---

### 创建启动文件

`launch/pubsub_launch.py`：

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 启动发布者
        Node(
            package='my_pubsub',
            executable='publisher',
            name='publisher_node',
            output='screen'
        ),
        
        # 启动订阅者
        Node(
            package='my_pubsub',
            executable='subscriber',
            name='subscriber_node',
            output='screen'
        ),
    ])
```

---

### 修改 setup.py

找到 `data_files`，修改为：

```python
data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    # 添加 launch 文件
    ('share/' + package_name + '/launch', ['launch/pubsub_launch.py']),
],
```

---

### 编译

```bash
cd ~/ros2_ws
colcon build --packages-select my_pubsub
source install/setup.bash
```

---

### 运行启动文件

```bash
ros2 launch my_pubsub pubsub_launch.py
```

!!! success "一次启动两个节点"
    现在发布者和订阅者会同时启动，
    
    输出也会显示在同一个终端里。

---

## 12.10 常见问题

### 问题1：订阅者收不到消息

!!! info "可能原因"
    1. **话题名不一致**
       - 检查发布者和订阅者的话题名
    
    2. **消息类型不一致**
       - 检查消息类型是否相同
    
    3. **没有 source**
       - 运行前要 source install/setup.bash

---

### 问题2：消息丢失

!!! info "可能原因"
    **QoS 策略问题**
    
    可以增大队列大小：
    
    ```python
    self.publisher_ = self.create_publisher(String, 'chatter', 100)
    ```

---

### 问题3：延迟很大

!!! info "可能原因"
    **发布频率太高**
    
    可以调整定时器周期：
    
    ```python
    self.timer = self.create_timer(0.1, self.timer_callback)  # 更快
    ```

---

## 12.11 QoS（服务质量）简介

### 什么是 QoS

!!! info "定义"
    QoS (Quality of Service) 定义了消息传输的质量策略。

---

### 常用的 QoS 配置

```python
from rclpy.qos import QoSProfile

# 创建 QoS 配置
qos = QoSProfile(depth=10)

# 使用 QoS 创建发布者
self.publisher_ = self.create_publisher(
    String,
    'chatter',
    qos
)
```

---

### QoS 预设配置

```python
from rclpy.qos import qos_profile_sensor_data

# 使用传感器数据的 QoS 配置
self.publisher_ = self.create_publisher(
    String,
    'chatter',
    qos_profile_sensor_data
)
```

!!! info "常用预设"
    - `qos_profile_sensor_data`：传感器数据
    - `qos_profile_system_default`：系统默认

---

## 本章小结

通过这一章，你应该已经掌握：

1. **发布者-订阅者模式**

    创建发布者节点

    创建订阅者节点

    通过话题通信

2. **消息类型的使用**

    String 消息

    Twist 消息

3. **命令行调试工具**

    ros2 topic list/info/echo/hz

    ros2 topic pub

    rqt_graph

4. **启动文件**

    一次启动多个节点

5. **QoS 基础**

    了解 QoS 的概念

---

## 下一章预告

下一章我们会做一个 **更有趣的项目**：

> **进阶项目——控制小海龟运动**

用代码控制小海龟画图，

学习如何实际应用发布者-订阅者模式。

