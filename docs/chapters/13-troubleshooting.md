# 第十三章:进阶项目——控制小海龟运动

用代码控制小海龟画图,这个比较有意思,也很实用。

通过这个项目,你会学到:

- 如何发布 Twist 消息控制机器人运动
- 如何调用服务
- 如何订阅位姿信息
- 如何实现更复杂的运动控制

---

## 13.1 了解小海龟的话题

先启动小海龟:

```bash
ros2 run turtlesim turtlesim_node
```

会弹出一个窗口,里面有只小海龟。

---

### 查看话题列表

```bash
ros2 topic list
```

你会看到这些话题:

```
/turtle1/cmd_vel          # 速度控制
/turtle1/color_sensor     # 颜色传感器
/turtle1/pose             # 位姿信息
/parameter_events
/rosout
```

---

### 最重要的话题: /turtle1/cmd_vel

这个话题用来控制小海龟移动。

查看消息类型:

```bash
ros2 topic info /turtle1/cmd_vel
```

输出:

```
Type: geometry_msgs/msg/Twist
```

看看这个消息的结构:

```bash
ros2 interface show geometry_msgs/msg/Twist
```

输出:

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

!!! info "理解 Twist 消息"
    - **linear**: 线速度(直线运动)
    - **angular**: 角速度(旋转运动)
    
    对于2D的小海龟:
    - `linear.x`: 控制前进/后退
    - `angular.z`: 控制左转/右转
    - 其他字段忽略

---

### 手动测试一下

试试用命令行发布消息:

```bash
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0}, angular: {z: 1.8}}"
```

!!! success "观察结果"
    小海龟会开始画圆!
    
    - `linear.x: 2.0` 让它向前走
    - `angular.z: 1.8` 让它同时转弯

按 `Ctrl+C` 停止。

---

## 13.2 项目一:画圆

现在用代码实现刚才的效果。

---

### 创建功能包

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python turtle_controller --dependencies rclpy geometry_msgs
```

---

### 编写画圆代码

创建 `turtle_controller/turtle_controller/draw_circle.py`:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DrawCircle(Node):
    def __init__(self):
        super().__init__('draw_circle')
        
        # 创建发布者
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        # 创建定时器,每0.1秒调用一次
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info('开始画圆')
    
    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 1.0   # 向前
        msg.angular.z = 1.0  # 同时转弯,就会画圆
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DrawCircle()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 停止时让海龟停下来
        stop_msg = Twist()
        node.publisher_.publish(stop_msg)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

### 配置 setup.py

编辑 `setup.py`,在 `entry_points` 里添加:

```python
entry_points={
    'console_scripts': [
        'draw_circle = turtle_controller.draw_circle:main',
    ],
},
```

---

### 编译并运行

```bash
cd ~/ros2_ws
colcon build --packages-select turtle_controller
source install/setup.bash
```

开两个终端:

**终端1**: 启动小海龟

```bash
ros2 run turtlesim turtlesim_node
```

**终端2**: 运行画圆程序

```bash
ros2 run turtle_controller draw_circle
```

!!! success "效果"
    小海龟开始画圆了!

想清除轨迹可以调用服务:

```bash
ros2 service call /clear std_srvs/srv/Empty
```

---

## 13.3 项目二:画正方形

现在我们做个更复杂的:让小海龟画正方形。

---

### 分析思路

画正方形需要:

1. 直走一段距离
2. 停下
3. 转90度
4. 重复4次

我们需要知道小海龟当前的位置,所以要订阅 `/turtle1/pose` 话题。

---

### 查看位姿消息

```bash
ros2 interface show turtlesim/msg/Pose
```

输出:

```
float32 x
float32 y
float32 theta      # 朝向角度
float32 linear_velocity
float32 angular_velocity
```

---

### 编写画正方形代码

创建 `turtle_controller/turtle_controller/draw_square.py`:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class DrawSquare(Node):
    def __init__(self):
        super().__init__('draw_square')
        
        # 发布者
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        # 订阅者
        self.subscription = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback, 10)
        
        # 状态变量
        self.current_pose = None
        self.state = 'move'  # 状态: move(直行), turn(转弯)
        self.target_distance = 2.0  # 每条边长度
        self.start_x = None
        self.start_y = None
        self.start_theta = None
        self.side_count = 0  # 已完成的边数
        
        # 定时器
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('开始画正方形')
    
    def pose_callback(self, msg):
        """接收位姿信息"""
        self.current_pose = msg
    
    def control_loop(self):
        """主控制循环"""
        if self.current_pose is None:
            return
        
        if self.side_count >= 4:
            # 画完了,停止
            self.stop()
            self.get_logger().info('正方形画完了!')
            self.timer.cancel()
            return
        
        if self.state == 'move':
            self.move_forward()
        elif self.state == 'turn':
            self.turn_90_degrees()
    
    def move_forward(self):
        """直线前进"""
        if self.start_x is None:
            # 记录起始位置
            self.start_x = self.current_pose.x
            self.start_y = self.current_pose.y
        
        # 计算已走距离
        distance = math.sqrt(
            (self.current_pose.x - self.start_x)**2 + 
            (self.current_pose.y - self.start_y)**2
        )
        
        if distance < self.target_distance:
            # 继续前进
            msg = Twist()
            msg.linear.x = 1.0
            self.publisher_.publish(msg)
        else:
            # 到达目标距离,切换到转弯状态
            self.stop()
            self.state = 'turn'
            self.start_x = None
            self.start_y = None
            self.start_theta = None
            self.get_logger().info(f'完成第 {self.side_count + 1} 条边')
    
    def turn_90_degrees(self):
        """转90度"""
        if self.start_theta is None:
            # 记录起始角度
            self.start_theta = self.current_pose.theta
        
        # 计算已转角度
        angle_diff = abs(self.current_pose.theta - self.start_theta)
        
        # 处理角度跳变(-π到π)
        if angle_diff > math.pi:
            angle_diff = 2 * math.pi - angle_diff
        
        if angle_diff < math.pi / 2 - 0.05:  # 90度,留点误差
            # 继续转
            msg = Twist()
            msg.angular.z = 0.5  # 转速
            self.publisher_.publish(msg)
        else:
            # 转完了,切换到前进状态
            self.stop()
            self.state = 'move'
            self.side_count += 1
            self.start_theta = None
    
    def stop(self):
        """停止运动"""
        msg = Twist()
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DrawSquare()
    
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

### 配置 setup.py

```python
entry_points={
    'console_scripts': [
        'draw_circle = turtle_controller.draw_circle:main',
        'draw_square = turtle_controller.draw_square:main',
    ],
},
```

---

### 编译并运行

```bash
cd ~/ros2_ws
colcon build --packages-select turtle_controller
source install/setup.bash
```

```bash
# 终端1
ros2 run turtlesim turtlesim_node

# 终端2
ros2 run turtle_controller draw_square
```

!!! success "效果"
    小海龟会画一个正方形!

---

## 13.4 项目三:键盘控制

再来个实用的:用键盘控制小海龟,类似游戏手柄。

---

### 安装依赖

需要用到 `pynput` 库来捕获键盘输入:

```bash
pip install pynput --break-system-packages
```

---

### 编写代码

创建 `turtle_controller/turtle_controller/keyboard_control.py`:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pynput import keyboard

class KeyboardControl(Node):
    def __init__(self):
        super().__init__('keyboard_control')
        
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        # 当前速度
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        
        # 速度增量
        self.speed_step = 0.5
        self.turn_step = 0.5
        
        # 定时发布速度
        self.timer = self.create_timer(0.1, self.publish_velocity)
        
        # 键盘监听器
        self.listener = keyboard.Listener(on_press=self.on_key_press)
        self.listener.start()
        
        self.get_logger().info('键盘控制已启动!')
        self.get_logger().info('使用方向键控制, 空格停止, ESC退出')
    
    def on_key_press(self, key):
        """按键处理"""
        try:
            if key == keyboard.Key.up:
                self.linear_speed += self.speed_step
                self.get_logger().info(f'加速: {self.linear_speed:.1f}')
            
            elif key == keyboard.Key.down:
                self.linear_speed -= self.speed_step
                self.get_logger().info(f'减速: {self.linear_speed:.1f}')
            
            elif key == keyboard.Key.left:
                self.angular_speed += self.turn_step
                self.get_logger().info(f'左转: {self.angular_speed:.1f}')
            
            elif key == keyboard.Key.right:
                self.angular_speed -= self.turn_step
                self.get_logger().info(f'右转: {self.angular_speed:.1f}')
            
            elif key == keyboard.Key.space:
                self.linear_speed = 0.0
                self.angular_speed = 0.0
                self.get_logger().info('停止')
            
            elif key == keyboard.Key.esc:
                self.get_logger().info('退出')
                rclpy.shutdown()
        
        except AttributeError:
            pass
    
    def publish_velocity(self):
        """定时发布速度"""
        msg = Twist()
        msg.linear.x = self.linear_speed
        msg.angular.z = self.angular_speed
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControl()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 停止海龟
        stop_msg = Twist()
        node.publisher_.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

### 配置并运行

```python
entry_points={
    'console_scripts': [
        'draw_circle = turtle_controller.draw_circle:main',
        'draw_square = turtle_controller.draw_square:main',
        'keyboard_control = turtle_controller.keyboard_control:main',
    ],
},
```

```bash
colcon build --packages-select turtle_controller
source install/setup.bash
ros2 run turtle_controller keyboard_control
```

!!! info "操作说明"
    - ↑ 加速
    - ↓ 减速
    - ← 左转
    - → 右转
    - 空格 停止
    - ESC 退出

---

## 13.5 进阶挑战

如果你已经掌握了上面的内容,可以试试这些:

---

### 挑战1: 画五角星

提示:

- 每次转144度(360/5 * 2)
- 需要5条边

---

### 挑战2: 跟随鼠标

提示:

- 订阅 `/turtle1/pose` 获取当前位置
- 计算到目标点的距离和角度
- 根据距离调整速度,根据角度调整转向

---

### 挑战3: 多只海龟协同

提示:

- 使用服务 `/spawn` 生成新海龟
- 每只海龟有独立的话题: `/turtle2/cmd_vel`
- 让两只海龟追逐或避开

---

## 13.6 常用服务调用

小海龟还提供了一些实用的服务:

---

### 清除轨迹

```bash
ros2 service call /clear std_srvs/srv/Empty
```

---

### 重置海龟

```bash
ros2 service call /reset std_srvs/srv/Empty
```

---

### 生成新海龟

```bash
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2, y: 2, theta: 0.2, name: 'turtle2'}"
```

---

### 杀死海龟

```bash
ros2 service call /kill turtlesim/srv/Kill "{name: 'turtle2'}"
```

---

### 设置画笔

```bash
ros2 service call /turtle1/set_pen turtlesim/srv/SetPen "{r: 255, g: 0, b: 0, width: 5, 'off': 0}"
```

参数说明:

- `r, g, b`: RGB颜色值(0-255)
- `width`: 线宽
- `off`: 1表示抬笔(不画线),0表示落笔

---

## 13.7 调试技巧

---

### 实时查看速度命令

```bash
ros2 topic echo /turtle1/cmd_vel
```

---

### 实时查看位姿

```bash
ros2 topic echo /turtle1/pose
```

---

### 查看节点关系图

```bash
ros2 run rqt_graph rqt_graph
```

---

### 查看话题发布频率

```bash
ros2 topic hz /turtle1/cmd_vel
```

---

## 本章小结

通过这一章的学习,你应该掌握了:

- ✅ 如何发布 Twist 消息控制运动
- ✅ 如何订阅位姿信息
- ✅ 如何实现状态机控制复杂运动
- ✅ 如何调用服务
- ✅ 如何处理键盘输入

!!! success "重要收获"
    这些技能不仅适用于小海龟,也是控制真实机器人的基础。
    
    - Twist 消息是 ROS 中最常用的速度控制消息
    - 状态机是机器人控制的常用模式
    - 订阅传感器数据 + 发布控制命令 = 机器人的基本工作流程

---

## 下一章预告

下一章我们会总结学习过程中最常遇到的问题和解决方案。

把这些坑都记下来,能帮你少走很多弯路!

