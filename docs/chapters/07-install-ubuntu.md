# 第七章：ROS2 支持哪些编程语言——选择适合你的

ROS2 支持多种编程语言，但官方主推的是两种：

- **Python（rclpy）**
- **C++（rclcpp）**

这一章我们详细聊聊这两种语言在 ROS2 中的应用。

---

## 7.1 官方支持的两种语言

### Python（rclpy）

#### 优点

!!! success "Python 的优势"
    - ✓ **语法简单**
      - 代码量少，可读性强
    
    - ✓ **开发快**
      - 不需要编译，改完直接运行
    
    - ✓ **调试方便**
      - 报错信息清晰，容易定位问题
    
    - ✓ **第三方库多**
      - NumPy、OpenCV、TensorFlow 等
      - 数据处理、图像处理、机器学习都很方便

---

#### 缺点

!!! warning "Python 的劣势"
    - ✗ **运行慢**
      - 解释型语言，性能比 C++ 差
    
    - ✗ **不支持实时系统**
      - 有 GIL（全局解释器锁）
      - 无法保证确定性响应时间

---

#### 适合场景

!!! info "推荐使用 Python 的情况"
    - ✓ 学习入门
    - ✓ 快速验证想法
    - ✓ 对性能要求不高的应用
    - ✓ 数据处理和分析
    - ✓ 深度学习相关应用

---

### C++（rclcpp）

#### 优点

!!! success "C++ 的优势"
    - ✓ **运行快**
      - 编译型语言，性能优秀
    
    - ✓ **性能好**
      - 可以充分利用硬件性能
    
    - ✓ **支持实时**
      - 可以用于实时控制系统
    
    - ✓ **底层驱动多是 C++ 写的**
      - 与硬件交互更直接

---

#### 缺点

!!! warning "C++ 的劣势"
    - ✗ **语法复杂**
      - 学习曲线陡峭
      - 指针、内存管理等概念容易出错
    
    - ✗ **开发慢**
      - 需要编译，改一点代码要重新编译
      - 编译时间长
    
    - ✗ **调试相对困难**
      - 报错信息有时不够清晰

---

#### 适合场景

!!! info "推荐使用 C++ 的情况"
    - ✓ 对性能要求高的应用
    - ✓ 实时控制系统
    - ✓ 硬件驱动开发
    - ✓ 商业产品

---

## 7.2 我的建议

### 新手先用 Python！

!!! success "推荐学习路径"
    **先用 Python 学习 ROS2 的概念和用法。**

---

#### 为什么这样建议？

1. **降低学习难度**
   - 不用同时学 ROS2 和 C++ 语法
   - 专注于理解 ROS2 的核心概念

2. **快速看到效果**
   - 写几行代码就能跑起来
   - 增强学习信心

3. **后续转 C++ 不难**
   - ROS2 的 Python 和 C++ API 设计类似
   - 概念清楚后，语法只是工具

---

### 不要一上来就学 C++

!!! danger "常见误区"
    别一上来就学 C++，
    
    容易被语法劝退，
    
    反而忽略了对 ROS2 本身的理解。

---

## 7.3 Python 和 C++ 在 ROS2 中的对比

### 代码对比

同一个功能，用两种语言实现：

---

#### Python 版本

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0
    
    def timer_callback(self):
        msg = String()
        msg.data = f'Hello {self.count}'
        self.publisher_.publish(msg)
        self.count += 1

def main():
    rclpy.init()
    node = PublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

#### C++ 版本

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class PublisherNode : public rclcpp::Node {
public:
    PublisherNode() : Node("publisher_node"), count_(0) {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&PublisherNode::timer_callback, this)
        );
    }

private:
    void timer_callback() {
        auto msg = std_msgs::msg::String();
        msg.data = "Hello " + std::to_string(count_);
        publisher_->publish(msg);
        count_++;
    }
    
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int count_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

---

### 可以看到：

!!! info "代码对比"
    - **Python 版本**
      - 代码更简洁
      - 可读性更强
      - 20 多行代码
    
    - **C++ 版本**
      - 代码更啰嗦
      - 需要包含头文件
      - 需要声明类型
      - 30 多行代码

---

## 7.4 性能对比

### 实际测试

!!! info "性能数据"
    对于简单的发布/订阅：
    - C++ 延迟约 **0.1 ms**
    - Python 延迟约 **1-2 ms**

---

### 什么时候性能差异重要？

!!! warning "需要考虑性能的场景"
    1. **高频控制**
       - 比如 1000 Hz 的控制循环
    
    2. **实时系统**
       - 工业机器人
       - 无人机姿态控制
    
    3. **大量数据处理**
       - 高速图像处理
       - 点云处理

---

### 什么时候性能差异不重要？

!!! success "Python 完全够用的场景"
    1. **低频任务**
       - 比如 10 Hz 的导航
    
    2. **非实时应用**
       - 数据记录
       - 用户界面
    
    3. **学习阶段**
       - 理解概念最重要

---

## 7.5 混合使用策略

### 最佳实践

!!! success "工程化方案"
    **Python 和 C++ 的节点可以在一个项目里共存！**

---

### 混合使用的典型方案

```
机器人系统架构：

底层控制（C++）
    ↓
    └─ 电机驱动节点（C++）
    └─ 传感器驱动节点（C++）
    └─ 实时控制节点（C++）

中间层（Python/C++ 混合）
    ↓
    └─ SLAM 节点（C++）
    └─ 导航节点（C++）
    └─ 图像处理节点（Python）

高层决策（Python）
    ↓
    └─ 任务规划节点（Python）
    └─ 用户界面节点（Python）
    └─ 数据记录节点（Python）
```

---

### 混合使用的优势

!!! info "各取所长"
    - **性能关键部分用 C++**
      - 保证实时性和效率
    
    - **快速开发部分用 Python**
      - 提高开发效率
      - 方便调试和修改

---

## 7.6 学习路径建议

### 阶段一：入门（1-2 个月）

!!! success "Python 入门"
    - 只用 Python
    - 学习 ROS2 核心概念
    - 完成基础项目

---

### 阶段二：进阶（2-3 个月）

!!! info "深入学习"
    - 继续用 Python
    - 学习更复杂的功能
    - 导航、SLAM、机械臂控制等

---

### 阶段三：优化（根据需求）

!!! tip "按需选择"
    - 如果需要性能
      - 学习 C++
      - 重写关键节点
    
    - 如果 Python 够用
      - 继续用 Python
      - 专注于应用开发

---

## 7.7 其他语言支持

### 社区支持的语言

除了 Python 和 C++，ROS2 社区还有一些其他语言的实现：

---

#### Rust

!!! info "新兴选择"
    - 性能接近 C++
    - 内存安全
    - 适合系统编程

**状态：** 社区开发中，不够成熟

---

#### Java

!!! info "企业级"
    - 适合企业应用
    - 跨平台

**状态：** 有官方支持，但不如 Python/C++ 完善

---

#### JavaScript/TypeScript

!!! info "Web 开发"
    - 适合 Web 界面开发
    - ros2djs、ros3djs

**状态：** 主要用于可视化，不用于核心开发

---

### 建议

!!! tip "语言选择"
    **对于学习和开发，还是建议专注于 Python 和 C++。**
    
    其他语言可以作为了解，
    
    但不要在学习阶段增加额外负担。

---

## 7.8 常见问题

### 问题1：我只会 Python，能找到工作吗？

!!! success "完全可以"
    - 很多公司的 ROS 项目主要用 Python
    - 特别是研究型岗位
    - 但如果会 C++，选择会更多

---

### 问题2：我只会 C++，可以吗？

!!! info "当然可以"
    - 很多底层开发必须用 C++
    - 但建议也了解 Python
    - 可以提高开发效率

---

### 问题3：我应该花多少时间学 C++？

!!! tip "建议"
    如果你的主要目标是学 ROS2：
    
    1. **先用 2-3 个月学好 ROS2（用 Python）**
    2. **然后根据需求决定是否学 C++**
    3. **如果要学 C++，再花 1-2 个月**

---

### 问题4：Python 节点和 C++ 节点能通信吗？

!!! success "完全可以"
    - ROS2 的通信是语言无关的
    - Python 节点和 C++ 节点可以无缝通信
    - 这也是混合开发的基础

---

## 7.9 代码风格建议

### Python 代码风格

遵循 **PEP 8** 规范：

```python
# 好的命名
class RobotController(Node):
    def __init__(self):
        self.max_speed = 2.0
    
    def calculate_velocity(self, distance):
        return min(distance, self.max_speed)

# 不好的命名
class rc(Node):
    def __init__(self):
        self.ms = 2.0
    
    def cv(self, d):
        return min(d, self.ms)
```

---

### C++ 代码风格

遵循 **Google C++ Style Guide** 或 **ROS2 Style Guide**：

```cpp
// 好的命名
class RobotController : public rclcpp::Node {
private:
    double max_speed_;
    
public:
    double calculateVelocity(double distance) {
        return std::min(distance, max_speed_);
    }
};

// 不好的命名
class RC : public rclcpp::Node {
private:
    double ms;
    
public:
    double cv(double d) {
        return std::min(d, ms);
    }
};
```

---

## 本章小结：

1. **ROS2 官方支持两种语言**

    Python（rclpy）：简单、开发快

    C++（rclcpp）：性能好、支持实时

2. **语言选择建议**

    新手先用 Python

    需要性能时用 C++

    可以混合使用

3. **学习路径**

    先用 Python 学 ROS2 概念

    根据需求决定是否学 C++

    不要在学习初期增加额外负担

4. **混合开发**

    Python 和 C++ 节点可以共存

    各取所长，提高开发效率

---

## 下一章预告

下一章我们会进入实战部分：

> **Ubuntu 系统安装教程——搭建环境**

手把手教你安装 Ubuntu 系统，

为学习 ROS2 做好准备。

