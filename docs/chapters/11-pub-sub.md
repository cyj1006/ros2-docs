# 第十一章：第一个 ROS2 程序——Hello World

!!! quote "学习规律"
    **学啥都得从 Hello World 开始。**

这一章我们来写第一个 ROS2 节点。

---

## 11.1 创建功能包

### 进入工作空间的 src 目录

```bash
cd ~/ros2_ws/src
```

---

### 创建包

```bash
ros2 pkg create --build-type ament_python hello_ros2 \
    --dependencies rclpy std_msgs
```

---

### 命令解释

!!! info "参数说明"
    - `--build-type ament_python`：创建 Python 包
    - `hello_ros2`：包的名字
    - `--dependencies rclpy std_msgs`：依赖的包
      - `rclpy`：ROS2 的 Python 客户端库
      - `std_msgs`：标准消息类型

---

### 创建后的目录结构

```
hello_ros2/
├── hello_ros2/         # Python 代码目录
│   └── __init__.py
├── resource/
│   └── hello_ros2
├── test/
├── package.xml         # 包的描述文件
├── setup.cfg
└── setup.py            # Python 包配置文件
```

---

## 11.2 编写节点代码

### 创建 Python 文件

在 `hello_ros2/hello_ros2/` 目录下创建 `hello_node.py`：

```bash
cd hello_ros2/hello_ros2
touch hello_node.py
```

---

### 编写代码

用你喜欢的编辑器打开 `hello_node.py`：

```bash
# VS Code
code hello_node.py

# 或者 nano
nano hello_node.py

# 或者 vim
vim hello_node.py
```

---

### 完整代码

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node


class HelloNode(Node):
    """Hello ROS2 节点"""
    
    def __init__(self):
        # 调用父类的构造函数，给节点起名字
        super().__init__('hello_node')
        
        # 打印启动信息
        self.get_logger().info('节点启动了！')
        
        # 创建定时器，每1秒调用一次回调函数
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        # 计数器
        self.count = 0
    
    def timer_callback(self):
        """定时器回调函数"""
        self.count += 1
        self.get_logger().info(f'Hello ROS2! 第 {self.count} 次问候')


def main(args=None):
    """主函数"""
    # 初始化 rclpy
    rclpy.init(args=args)
    
    # 创建节点
    node = HelloNode()
    
    try:
        # 保持节点运行，处理回调
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 用户按 Ctrl+C 时
        pass
    finally:
        # 清理资源
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## 11.3 代码详解

### 第一部分：导入库

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
```

!!! info "说明"
    - `#!/usr/bin/env python3`：指定用 Python3 运行
    - `rclpy`：ROS2 的 Python 客户端库
    - `Node`：节点的基类

---

### 第二部分：定义节点类

```python
class HelloNode(Node):
    """Hello ROS2 节点"""
```

!!! info "说明"
    - 继承自 `Node` 类
    - 所有 ROS2 节点都要继承 `Node`

---

### 第三部分：初始化函数

```python
def __init__(self):
    # 调用父类的构造函数，给节点起名字
    super().__init__('hello_node')
```

!!! warning "必须做"
    - `super().__init__('hello_node')`：**必须调用**
    - `'hello_node'`：节点的名字

---

```python
    # 打印启动信息
    self.get_logger().info('节点启动了！')
```

!!! info "日志系统"
    - `self.get_logger()`：获取节点的日志记录器
    - `.info()`：打印信息级别的日志
    - 还有：`.debug()`, `.warn()`, `.error()`

---

```python
    # 创建定时器，每1秒调用一次回调函数
    self.timer = self.create_timer(1.0, self.timer_callback)
```

!!! info "定时器"
    - `create_timer(周期, 回调函数)`
    - `1.0`：1 秒
    - `self.timer_callback`：回调函数名

---

```python
    # 计数器
    self.count = 0
```

!!! info "成员变量"
    用来记录问候的次数。

---

### 第四部分：回调函数

```python
def timer_callback(self):
    """定时器回调函数"""
    self.count += 1
    self.get_logger().info(f'Hello ROS2! 第 {self.count} 次问候')
```

!!! info "说明"
    - 每秒被调用一次
    - 计数器加 1
    - 打印问候信息

---

### 第五部分：主函数

```python
def main(args=None):
    # 初始化 rclpy
    rclpy.init(args=args)
```

!!! warning "必须做"
    `rclpy.init()`：**必须调用**，初始化 ROS2 系统

---

```python
    # 创建节点
    node = HelloNode()
```

!!! info "实例化"
    创建我们定义的节点对象。

---

```python
    try:
        # 保持节点运行，处理回调
        rclpy.spin(node)
```

!!! info "spin 的作用"
    - 让节点保持运行
    - 不断处理回调函数
    - 如果没有 spin，程序会立刻结束

---

```python
    except KeyboardInterrupt:
        # 用户按 Ctrl+C 时
        pass
```

!!! info "优雅退出"
    捕获 `Ctrl+C`，避免报错信息。

---

```python
    finally:
        # 清理资源
        node.destroy_node()
        rclpy.shutdown()
```

!!! warning "必须做"
    - `destroy_node()`：销毁节点
    - `shutdown()`：关闭 ROS2 系统

---

```python
if __name__ == '__main__':
    main()
```

!!! info "入口点"
    如果直接运行这个文件，就调用 `main()` 函数。

---

## 11.4 配置入口点

### 编辑 setup.py

打开 `hello_ros2/setup.py`：

```bash
cd ~/ros2_ws/src/hello_ros2
nano setup.py
```

---

### 找到 entry_points

找到这一段：

```python
entry_points={
    'console_scripts': [
    ],
},
```

---

### 添加我们的节点

修改为：

```python
entry_points={
    'console_scripts': [
        'hello_node = hello_ros2.hello_node:main',
    ],
},
```

!!! info "格式说明"
    `'命令名 = 包名.文件名:函数名'`
    
    - `hello_node`：以后运行时用的命令名
    - `hello_ros2.hello_node`：Python 模块路径
    - `main`：要调用的函数

---

## 11.5 编译和运行

### 回到工作空间根目录

```bash
cd ~/ros2_ws
```

---

### 编译包

```bash
colcon build --packages-select hello_ros2
```

!!! info "编译选项"
    - `--packages-select hello_ros2`：只编译这一个包
    - 如果不加，会编译所有包

---

### 加载环境

```bash
source install/setup.bash
```

!!! danger "必须做"
    编译完后 **必须 source**，否则找不到新编译的包。

---

### 运行节点

```bash
ros2 run hello_ros2 hello_node
```

---

### 应该看到

```
[INFO] [节点名]: 节点启动了！
[INFO] [节点名]: Hello ROS2! 第 1 次问候
[INFO] [节点名]: Hello ROS2! 第 2 次问候
[INFO] [节点名]: Hello ROS2! 第 3 次问候
...
```

!!! success "成功"
    每秒打印一条消息！

---

### 停止节点

按 `Ctrl+C` 停止节点。

---

## 11.6 查看节点信息

### 另开一个终端

在节点运行时，打开另一个终端。

---

### 查看运行的节点

```bash
ros2 node list
```

!!! success "输出"
    ```
    /hello_node
    ```

---

### 查看节点详细信息

```bash
ros2 node info /hello_node
```

!!! info "会显示"
    - 订阅的话题
    - 发布的话题
    - 提供的服务
    - 使用的参数

---

## 11.7 常见错误

### 错误1：找不到包

```
Package 'hello_ros2' not found
```

!!! tip "解决方法"
    ```bash
    # 重新 source
    source ~/ros2_ws/install/setup.bash
    
    # 检查包是否编译成功
    ros2 pkg list | grep hello
    ```

---

### 错误2：找不到可执行文件

```
No executable found
```

!!! tip "解决方法"
    1. 检查 `setup.py` 的 `entry_points` 是否配置正确
    2. 重新编译：
       ```bash
       cd ~/ros2_ws
       colcon build --packages-select hello_ros2
       source install/setup.bash
       ```

---

### 错误3：import 错误

```
ModuleNotFoundError: No module named 'rclpy'
```

!!! tip "解决方法"
    ```bash
    # source ROS2 环境
    source /opt/ros/humble/setup.bash
    ```

---

### 错误4：权限问题

```
Permission denied
```

!!! tip "解决方法"
    ```bash
    # 给文件添加执行权限
    chmod +x hello_ros2/hello_node.py
    ```

---

## 11.8 改进版本：带参数的节点

### 修改代码

我们可以让问候的频率可配置：

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node


class HelloNode(Node):
    """Hello ROS2 节点（带参数版本）"""
    
    def __init__(self):
        super().__init__('hello_node')
        
        # 声明参数
        self.declare_parameter('timer_period', 1.0)
        self.declare_parameter('greeting_text', 'Hello ROS2!')
        
        # 获取参数值
        timer_period = self.get_parameter('timer_period').value
        greeting_text = self.get_parameter('greeting_text').value
        
        self.get_logger().info(f'节点启动！周期={timer_period}秒')
        
        # 使用参数创建定时器
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.count = 0
        self.greeting_text = greeting_text
    
    def timer_callback(self):
        """定时器回调函数"""
        self.count += 1
        self.get_logger().info(f'{self.greeting_text} 第 {self.count} 次')


def main(args=None):
    rclpy.init(args=args)
    node = HelloNode()
    
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

### 重新编译

```bash
cd ~/ros2_ws
colcon build --packages-select hello_ros2
source install/setup.bash
```

---

### 运行时设置参数

```bash
# 默认运行
ros2 run hello_ros2 hello_node

# 修改问候周期为0.5秒
ros2 run hello_ros2 hello_node --ros-args -p timer_period:=0.5

# 同时修改问候文字
ros2 run hello_ros2 hello_node --ros-args \
    -p timer_period:=0.5 \
    -p greeting_text:="你好 ROS2!"
```

---

### 运行时查看参数

另开终端：

```bash
# 列出节点的参数
ros2 param list /hello_node

# 查看参数值
ros2 param get /hello_node timer_period

# 修改参数值
ros2 param set /hello_node timer_period 2.0
```

---

## 11.9 完整开发流程总结

### 步骤回顾

!!! success "标准流程"
    1. **创建功能包**
       ```bash
       ros2 pkg create --build-type ament_python 包名 --dependencies rclpy
       ```
    
    2. **编写代码**
       - 在 `包名/包名/` 下创建 Python 文件
       - 写节点类，继承 `Node`
       - 写 `main()` 函数
    
    3. **配置 setup.py**
       - 在 `entry_points` 里添加入口点
    
    4. **编译**
       ```bash
       cd ~/ros2_ws
       colcon build --packages-select 包名
       ```
    
    5. **加载环境**
       ```bash
       source install/setup.bash
       ```
    
    6. **运行**
       ```bash
       ros2 run 包名 节点名
       ```

---

## 本章小结

通过这一章，你应该已经：

1. **创建了第一个 ROS2 包**

    使用 `ros2 pkg create` 命令

2. **编写了第一个 ROS2 节点**

    定义节点类

    使用定时器

    使用日志系统

3. **理解了节点的基本结构**

    `__init__` 初始化

    回调函数

    `main()` 函数

4. **学会了编译和运行**

    `colcon build`

    `source install/setup.bash`

    `ros2 run`

5. **学会了使用参数**

    声明参数

    获取参数
    
    运行时修改参数

---

## 下一章预告

下一章我们会做一个 **更实用的项目**：

> **项目实战——发布者与订阅者通信**

学习节点之间如何通过话题进行通信，

这是 ROS2 中最常用的通信方式。

