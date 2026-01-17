# 第六章：开发工具介绍——认识你的"武器库"

!!! quote "工欲善其事"
    **工欲善其事，必先利其器。**

ROS2 提供了丰富的开发工具，

熟悉这些工具能让你的开发效率大大提高。

---

## 6.1 命令行工具

ROS2 自带一套命令行工具，用得最多的几个：

---

### ros2 run - 运行节点

#### 基本用法

```bash
ros2 run 包名 节点名
```

---

#### 实例

```bash
# 运行小海龟节点
ros2 run turtlesim turtlesim_node

# 运行键盘控制节点
ros2 run turtlesim turtle_teleop_key
```

---

### ros2 node - 节点管理

#### 常用命令

```bash
# 列出所有运行中的节点
ros2 node list

# 查看节点详细信息
ros2 node info /节点名
```

---

#### 实例

```bash
# 查看当前有哪些节点在运行
ros2 node list

# 查看小海龟节点的详细信息
ros2 node info /turtlesim
```

!!! tip "调试利器"
    这个命令可以看到节点的：
    - 订阅了哪些话题
    - 发布了哪些话题
    - 提供了哪些服务
    - 使用了哪些参数

---

### ros2 topic - 话题管理

#### 常用命令

```bash
# 列出所有话题
ros2 topic list

# 实时查看话题内容（调试神器）
ros2 topic echo /话题名

# 查看话题发布频率
ros2 topic hz /话题名

# 手动发布消息
ros2 topic pub /话题名 类型 "数据"

# 查看话题信息
ros2 topic info /话题名
```

---

#### 实例

```bash
# 查看小海龟的速度指令
ros2 topic echo /turtle1/cmd_vel

# 查看位置话题的发布频率
ros2 topic hz /turtle1/pose

# 手动发布速度指令让小海龟移动
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```

!!! success "调试技巧"
    `ros2 topic echo` 是调试时最常用的命令，
    
    可以实时看到数据流动，非常直观。

---

### ros2 service - 服务管理

#### 常用命令

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
# 调用清除轨迹的服务
ros2 service call /clear std_srvs/srv/Empty

# 重置小海龟位置
ros2 service call /reset std_srvs/srv/Empty

# 生成新的海龟
ros2 service call /spawn turtlesim/srv/Spawn \
    "{x: 2.0, y: 2.0, theta: 0.0, name: 'turtle2'}"
```

---

### ros2 launch - 启动文件

#### 基本用法

```bash
ros2 launch 包名 xxx.launch.py
```

!!! info "launch 文件的作用"
    一次启动多个节点，配置参数，非常方便。

---

#### 实例

```bash
# 启动 navigation2 导航
ros2 launch nav2_bringup navigation_launch.py
```

---

### ros2 bag - 数据录制

#### 常用命令

```bash
# 录制所有话题
ros2 bag record -a

# 录制指定话题
ros2 bag record /topic1 /topic2

# 回放录制的数据
ros2 bag play 文件夹名

# 查看bag文件信息
ros2 bag info 文件夹名
```

---

#### 实例

```bash
# 录制小海龟的所有数据
ros2 bag record -a

# 回放刚才录制的数据
ros2 bag play rosbag2_2026_01_15-10_30_25
```

!!! tip "应用场景"
    - 记录实验数据用于分析
    - 离线调试
    - 回放测试场景

---

### ros2 param - 参数管理

#### 常用命令

```bash
# 列出节点的所有参数
ros2 param list

# 获取参数值
ros2 param get /节点名 参数名

# 设置参数值
ros2 param set /节点名 参数名 值

# 保存参数到文件
ros2 param dump /节点名
```

---

#### 实例

```bash
# 查看小海龟节点的参数
ros2 param list /turtlesim

# 修改背景颜色
ros2 param set /turtlesim background_r 255
```

---

### ros2 interface - 接口查看

#### 常用命令

```bash
# 查看消息结构
ros2 interface show geometry_msgs/msg/Twist

# 列出所有消息类型
ros2 interface list
```

---

#### 实例

```bash
# 查看 Twist 消息的结构
ros2 interface show geometry_msgs/msg/Twist
```

输出：

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
    了解消息结构，方便编程时使用。

---

## 6.2 可视化工具

### RViz2 - 3D 可视化工具

#### 简介

!!! info "核心工具"
    RViz2 是 **最常用的可视化工具**，可以显示：
    - 机器人模型
    - 传感器数据
    - 路径规划结果
    - TF 坐标系
    - 地图

---

#### 启动命令

```bash
rviz2
```

---

#### 主要功能

1. **显示机器人模型（URDF）**
   
    看机器人长什么样

    查看关节运动

2. **显示传感器数据**

    激光雷达点云

    摄像头图像

    深度相机数据

3. **显示导航信息**
   
    地图

    路径规划

    机器人位置

4. **显示坐标变换（TF）**

    各个坐标系的关系

    实时更新

---

#### 使用技巧

!!! tip "调试建议"
    - 左侧面板添加你想显示的数据类型
    - 调整视角方便观察
    - 保存配置文件，下次直接加载

---

### Gazebo - 仿真器

#### 简介

!!! info "仿真环境"
    Gazebo 是 ROS2 生态里的标准仿真器，可以：
    - 创建虚拟环境
    - 模拟物理效果
    - 测试机器人
    - 不需要真实硬件

---

#### 为什么需要仿真

!!! success "仿真的优势"
    - ✓ 真机器人太贵，撞坏了心疼
    - ✓ 在仿真里可以随便试错
    - ✓ 可以模拟各种极端情况
    - ✓ 开发效率高

---

#### 启动方式

```bash
# 直接启动 Gazebo
gazebo

# 启动空白世界
ros2 launch gazebo_ros gazebo.launch.py

# 启动带机器人的仿真
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

---

#### 主要功能

1. **物理仿真**

    重力

    碰撞

    摩擦力

2. **传感器仿真**

    激光雷达

    摄像头

    IMU

3. **环境建模**

    可以自己创建场景

    添加障碍物

---

### rqt - 图形化工具集

#### 简介

!!! info "工具集合"
    rqt 是一系列图形化工具的集合。

---

#### 常用的 rqt 工具

##### 1. rqt_graph - 查看节点关系

```bash
ros2 run rqt_graph rqt_graph
```

!!! success "可视化神器"
    可以一眼看出：
    - 有哪些节点在运行
    - 它们之间通过什么话题通信
    - 数据流向

---

##### 2. rqt_plot - 实时绘图

```bash
ros2 run rqt_plot rqt_plot
```

!!! info "用途"
    实时画出话题数据的曲线图，
    
    比如速度、位置随时间的变化。

---

##### 3. rqt_console - 日志查看

```bash
ros2 run rqt_console rqt_console
```

!!! info "用途"
    查看所有节点的日志输出，
    
    方便调试。

---

##### 4. rqt_image_view - 图像显示

```bash
ros2 run rqt_image_view rqt_image_view
```

!!! info "用途"
    显示摄像头话题的图像。

---

## 6.3 构建工具

### colcon - 编译工具

#### 简介

!!! info "官方构建工具"
    colcon 是 ROS2 的官方编译工具，
    
    用来编译你写的功能包。

---

#### 常用命令

```bash
# 编译所有包
colcon build

# 只编译指定的包
colcon build --packages-select 包名

# 编译指定包及其依赖
colcon build --packages-up-to 包名

# 并行编译（加快速度）
colcon build --parallel-workers 4

# 显示详细输出
colcon build --event-handlers console_direct+
```

---

#### 使用流程

```bash
# 1. 进入工作空间
cd ~/ros2_ws

# 2. 编译
colcon build

# 3. 加载环境（很重要！）
source install/setup.bash
```

!!! danger "常见错误"
    编译完后忘记 `source install/setup.bash`，
    
    导致找不到包。

---

#### 优化技巧

```bash
# 只编译修改过的包
colcon build --packages-select my_package

# 清理后重新编译
rm -rf build install log
colcon build
```

---

## 6.4 代码编辑器

### VS Code（强烈推荐）

#### 为什么推荐 VS Code

!!! success "优点"
    - ✓ 免费开源
    - ✓ 功能强大
    - ✓ 插件丰富
    - ✓ 跨平台
    - ✓ 对 ROS2 支持好

---

#### 必装插件

##### 1. Python

- 代码补全
- 语法检查
- 调试支持

---

##### 2. C/C++

- 如果要写 C++ 节点

---

##### 3. ROS

- ROS 语法高亮
- 自动补全 ROS 命令

---

##### 4. 其他推荐

- **Markdown All in One**：写文档
- **GitLens**：Git 可视化
- **Better Comments**：注释美化

---

#### 安装方法

```bash
# 方法一：官网下载
# https://code.visualstudio.com

# 方法二：命令行安装
sudo snap install code --classic
```

---

#### 使用技巧

```bash
# 在当前目录打开 VS Code
code .

# 打开特定文件
code file.py
```

---

### 其他编辑器选择

#### vim / neovim

!!! info "适合人群"
    - 喜欢纯键盘操作
    - 想要极致的编辑效率

---

#### PyCharm

!!! info "适合人群"
    - 主要写 Python
    - 需要强大的调试功能

---

#### CLion

!!! info "适合人群"
    - 主要写 C++
    - 需要 IDE 级别的支持

---

## 6.5 终端工具

### Terminator（推荐）

#### 为什么需要 Terminator

!!! info "多终端管理"
    ROS2 开发经常要同时开好几个终端，
    
    Terminator 可以在一个窗口里分割出多个终端。

---

#### 安装

```bash
sudo apt install terminator
```

---

#### 常用快捷键

| 快捷键 | 功能 |
|--------|------|
| `Ctrl+Shift+E` | 垂直分割 |
| `Ctrl+Shift+O` | 水平分割 |
| `Ctrl+Shift+W` | 关闭当前终端 |
| `Alt+方向键` | 切换终端 |

---

### tmux

!!! info "高级选择"
    如果需要更强大的终端管理，
    
    可以学习 tmux。

---

## 6.6 调试工具

### gdb - C++ 调试器

```bash
# 调试程序
gdb ./程序名

# 常用命令
(gdb) break main        # 设置断点
(gdb) run               # 运行
(gdb) next              # 单步执行
(gdb) print variable    # 打印变量
```

---

### pdb - Python 调试器

```python
# 在代码中插入断点
import pdb; pdb.set_trace()

# 或者用 breakpoint()（Python 3.7+）
breakpoint()
```

---

### VS Code 调试

!!! success "推荐方式"
    VS Code 的图形化调试非常方便，
    
    比命令行调试直观多了。

---

## 6.7 工具使用技巧总结

### 命令行工具技巧

!!! tip "效率提升"
    1. **多用 Tab 补全**

        减少输入错误
    
    2. **善用历史命令**

        上下箭头翻历史

        `Ctrl+R` 搜索历史命令
    
    3. **使用别名**

        在 `~/.bashrc` 里设置常用命令的别名

---

### 可视化工具技巧

!!! tip "调试建议"
    1. **RViz2 保存配置**

        配置好一次，以后直接加载
    
    2. **rqt_graph 查看数据流**

        遇到通信问题先看图
    
    3. **录制数据包**

        重要实验记得录制

---

## 本章小结

通过这一章，你应该已经认识了：

1. **命令行工具**

    ros2 run、node、topic、service、launch、bag

2. **可视化工具**

    RViz2、Gazebo、rqt 系列

3. **构建工具**

    colcon

4. **编辑器**

    VS Code（推荐）

5. **其他工具**

    Terminator、gdb、pdb

---

## 下一章预告

下一章我们会讲：

> **ROS2 支持哪些编程语言——选择适合你的**

帮你了解 Python 和 C++ 在 ROS2 中的应用，

以及如何选择适合自己的语言。

