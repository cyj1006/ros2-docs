# 第十四章:常见问题与解决方案——遇到问题别慌

学习 ROS2 的过程中,肯定会遇到各种问题。

这一章我把自己踩过的坑、学弟学妹们经常问的问题,都整理了一遍。

!!! info "使用建议"
    建议把这一章加入收藏,遇到问题时回来翻一翻,  
    大概率能找到解决办法。

---

## 14.1 环境相关问题

这类问题最常见,也最让新手抓狂。

---

### 问题1: 找不到包

**现象**:

```bash
ros2 run my_package my_node
# 输出: Package 'my_package' not found
```

**原因**: 环境变量没有加载

**解决方案**:

```bash
# 先加载 ROS2 环境
source /opt/ros/humble/setup.bash

# 再加载自己的工作空间
source ~/ros2_ws/install/setup.bash

# 验证是否成功
ros2 pkg list | grep my_package
```

!!! warning "永久解决"
    每次开新终端都要 source 很烦?加到 `.bashrc` 里:
    
    ```bash
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ```

---

### 问题2: source 了还是找不到

**可能原因1**: 编译后没有 source

```bash
cd ~/ros2_ws
colcon build --packages-select my_package

# ⚠️ 编译完一定要重新 source
source install/setup.bash
```

**可能原因2**: setup.py 配置错误

检查 `entry_points` 是否正确:

```python
entry_points={
    'console_scripts': [
        'my_node = my_package.my_node:main',  # 格式要对
    ],
},
```

**可能原因3**: 包名和节点名搞混了

```bash
# 错误
ros2 run my_node my_package  # 顺序反了

# 正确
ros2 run my_package my_node  # 包名在前,节点在后
```

---

### 问题3: Python 包导入失败

**现象**:

```python
ModuleNotFoundError: No module named 'xxx'
```

**解决方案**:

```bash
# 安装 Python 包时一定要加 --break-system-packages
pip install xxx --break-system-packages

# 或者用 apt 安装(推荐)
sudo apt install python3-xxx
```

---

## 14.2 编译相关问题

---

### 问题4: colcon build 失败

**情况1**: 缓存问题

```bash
cd ~/ros2_ws

# 清除所有缓存
rm -rf build install log

# 重新编译
colcon build
```

---

**情况2**: 依赖缺失

```bash
# 安装依赖
rosdep install --from-paths src --ignore-src -r -y
```

如果 rosdep 也报错,先更新它:

```bash
sudo rosdep init
rosdep update
```

---

**情况3**: setup.py 错误

常见错误:

```python
# ❌ 错误:忘记导入
from setuptools import setup

# ✅ 正确:完整导入
from setuptools import setup, find_packages
```

```python
# ❌ 错误:包名和文件夹名不一致
setup(
    name='my_pkg',  # setup.py 里写的
    packages=['my_package']  # 实际文件夹名
)

# ✅ 正确:一致
setup(
    name='my_package',
    packages=['my_package']
)
```

---

**情况4**: CMakeLists.txt 错误 (C++ 包)

如果是 C++ 包,检查 `CMakeLists.txt`:

```cmake
# 常见问题:忘记添加可执行文件
add_executable(my_node src/my_node.cpp)

# 或忘记链接依赖
ament_target_dependencies(my_node rclcpp)

# 或忘记安装
install(TARGETS
  my_node
  DESTINATION lib/${PROJECT_NAME}
)
```

---

### 问题5: 只想编译特定的包

```bash
# 只编译 my_package
colcon build --packages-select my_package

# 编译 my_package 及其依赖
colcon build --packages-up-to my_package

# 跳过某个包
colcon build --packages-skip bad_package
```

---

## 14.3 运行相关问题

---

### 问题6: 话题收不到消息

**步骤1**: 检查话题是否存在

```bash
ros2 topic list
```

---

**步骤2**: 检查是否有发布者

```bash
ros2 topic info /my_topic
```

输出应该有:

```
Publisher count: 1  # 至少有一个发布者
Subscriber count: 1
```

---

**步骤3**: 检查消息类型是否匹配

```bash
# 发布者的消息类型
ros2 topic info /my_topic

# 订阅者代码里的类型要一致
self.subscription = self.create_subscription(
    String,  # 这里的类型要匹配
    '/my_topic',
    self.callback,
    10
)
```

---

**步骤4**: 检查是否有数据

```bash
ros2 topic echo /my_topic
```

如果一直没输出,说明发布者没在发数据。

---

**步骤5**: 检查 QoS 设置

这是个比较隐蔽的问题:

```python
# 如果发布者用的是可靠模式
self.publisher_ = self.create_publisher(
    String, 
    '/my_topic', 
    qos_profile=rclpy.qos.QoSProfile(
        reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
        depth=10
    )
)

# 订阅者也要用可靠模式,否则收不到
```

通常情况下,用默认设置就行,别改 QoS。

---

### 问题7: 节点启动后立即退出

**原因1**: 没有 `rclpy.spin()`

```python
def main():
    rclpy.init()
    node = MyNode()
    
    # ❌ 缺少这一行,节点会立即退出
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()
```

---

**原因2**: 代码有异常

看日志:

```bash
ros2 run my_package my_node

# 或者查看详细日志
ros2 run my_package my_node --ros-args --log-level DEBUG
```

---

### 问题8: 回调函数不执行

**检查定时器是否创建**:

```python
# ❌ 忘记保存定时器对象,会被垃圾回收
self.create_timer(1.0, self.callback)

# ✅ 保存引用
self.timer = self.create_timer(1.0, self.callback)
```

---

**检查回调函数签名**:

```python
# 订阅者回调必须接收一个参数
def callback(self, msg):  # msg 不能省
    pass

# 定时器回调不需要参数
def timer_callback(self):
    pass
```

---

## 14.4 小海龟相关问题

---

### 问题9: 小海龟窗口是黑屏

**原因**: 虚拟机没开 3D 加速

**解决**:

1. 关闭虚拟机
2. 虚拟机设置 → 显示 → 勾选"加速 3D 图形"
3. 重启虚拟机

---

### 问题10: Gazebo 黑屏或闪退

**检查显卡驱动**:

```bash
glxinfo | grep "OpenGL version"
```

如果报错 `glxinfo: command not found`:

```bash
sudo apt install mesa-utils
```

---

**虚拟机设置**:

- 显存给到最大(256MB 或更多)
- 3D 加速必须开启
- 显卡控制器选 VMSVGA

---

**如果还是不行**,试试无 GUI 模式:

```bash
# 启动 Gazebo 时加 --headless
gazebo --verbose --headless
```

---

### 问题11: 小海龟不动

**检查是否发送了速度命令**:

```bash
ros2 topic echo /turtle1/cmd_vel
```

**检查速度值是否为0**:

```python
# ❌ 没有设置速度
msg = Twist()  # 默认全是0,当然不会动

# ✅ 设置速度
msg = Twist()
msg.linear.x = 1.0
```

---

## 14.5 权限相关问题

---

### 问题12: Permission denied

**现象1**: 编译时报错

```bash
# 工作空间权限不对
sudo chown -R $USER:$USER ~/ros2_ws
```

---

**现象2**: 无法执行 Python 文件

```bash
# 给执行权限
chmod +x ~/ros2_ws/src/my_package/my_package/my_node.py

# 或者编译时自动处理(在 setup.py 里已经设置了)
```

---

## 14.6 网络相关问题

---

### 问题13: 多台电脑的节点看不到对方

**检查网络**:

```bash
# 互相 ping 一下
ping 对方IP
```

---

**检查 ROS_DOMAIN_ID**:

```bash
# 查看当前设置
echo $ROS_DOMAIN_ID

# 设置相同的 ID(0-101)
export ROS_DOMAIN_ID=10

# 永久设置
echo "export ROS_DOMAIN_ID=10" >> ~/.bashrc
```

---

**检查防火墙**:

```bash
# Ubuntu 关闭防火墙(测试用)
sudo ufw disable

# 生产环境应该开放特定端口
sudo ufw allow 7400:7500/udp
sudo ufw allow 7400:7500/tcp
```

---

## 14.7 调试技巧

---

### 技巧1: 打印日志

```python
# 不同级别的日志
self.get_logger().debug('调试信息')
self.get_logger().info('普通信息')
self.get_logger().warn('警告')
self.get_logger().error('错误')
self.get_logger().fatal('致命错误')
```

查看日志:

```bash
# 运行时设置日志级别
ros2 run my_package my_node --ros-args --log-level DEBUG
```

---

### 技巧2: 查看节点信息

```bash
# 列出所有节点
ros2 node list

# 查看节点详细信息
ros2 node info /my_node

# 会显示:
# - 订阅的话题
# - 发布的话题
# - 提供的服务
# - 使用的动作
```

---

### 技巧3: 查看话题信息

```bash
# 列出所有话题
ros2 topic list

# 话题详细信息
ros2 topic info /my_topic

# 实时查看数据
ros2 topic echo /my_topic

# 查看发布频率
ros2 topic hz /my_topic

# 查看带宽
ros2 topic bw /my_topic
```

---

### 技巧4: 可视化节点关系

```bash
# 图形化显示
ros2 run rqt_graph rqt_graph

# 或者用命令行
ros2 node list
ros2 topic list
```

---

### 技巧5: 录制和回放数据

**录制**:

```bash
# 录制所有话题
ros2 bag record -a

# 只录制特定话题
ros2 bag record /turtle1/cmd_vel /turtle1/pose

# 指定输出文件
ros2 bag record -o my_data /my_topic
```

**回放**:

```bash
ros2 bag play my_data
```

这在调试时非常有用,可以反复重现问题。

---

### 技巧6: 手动发布消息

```bash
# 测试订阅者是否工作
ros2 topic pub /my_topic std_msgs/msg/String "data: 'test'"

# 发布 Twist 消息
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0}, angular: {z: 1.8}}"
```

---

### 技巧7: 调用服务

```bash
# 列出服务
ros2 service list

# 查看服务类型
ros2 service type /my_service

# 调用服务
ros2 service call /clear std_srvs/srv/Empty
```

---

## 14.8 常用命令速查表

### 节点相关

```bash
ros2 node list                    # 列出节点
ros2 node info /节点名             # 节点详情
ros2 run 包名 节点名               # 运行节点
```

---

### 话题相关

```bash
ros2 topic list                   # 列出话题
ros2 topic echo /话题名            # 查看数据
ros2 topic info /话题名            # 话题信息
ros2 topic hz /话题名              # 发布频率
ros2 topic bw /话题名              # 带宽
ros2 topic pub /话题名 类型 数据   # 发布消息
```

---

### 服务相关

```bash
ros2 service list                 # 列出服务
ros2 service type /服务名          # 服务类型
ros2 service call /服务名 类型     # 调用服务
```

---

### 参数相关

```bash
ros2 param list                   # 列出参数
ros2 param get /节点名 参数名      # 获取参数
ros2 param set /节点名 参数名 值   # 设置参数
ros2 param dump /节点名            # 导出参数
```

---

### 包相关

```bash
ros2 pkg list                     # 列出所有包
ros2 pkg list | grep xxx          # 搜索包
ros2 pkg create                   # 创建包
ros2 pkg executables 包名         # 列出包的节点
```

---

### 接口相关

```bash
ros2 interface list               # 列出接口
ros2 interface show 类型名        # 查看接口定义
ros2 interface package 包名       # 包的接口
```

---

### 编译相关

```bash
colcon build                                    # 编译所有
colcon build --packages-select 包名             # 编译指定包
colcon build --packages-up-to 包名              # 编译包及依赖
colcon build --packages-skip 包名               # 跳过某包
colcon build --symlink-install                 # 符号链接(Python推荐)
```

---

## 14.9 问题排查流程

遇到问题时,按这个流程走:

---

### 第1步: 看报错信息

**仔细看报错**,往往已经告诉你问题在哪了。

```
Package 'xxx' not found  → 包没找到,检查环境
No module named 'xxx'    → Python包没装
Permission denied        → 权限问题
```

---

### 第2步: 检查环境

```bash
# 环境加载了吗?
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# 包存在吗?
ros2 pkg list | grep my_package
```

---

### 第3步: 检查代码

- setup.py 配置对吗?
- 消息类型匹配吗?
- 语法错误?

---

### 第4步: 查看日志

```bash
ros2 run my_package my_node --ros-args --log-level DEBUG
```

---

### 第5步: 搜索

- Google: `ros2 + 你的错误信息`
- ROS Answers: https://answers.ros.org
- GitHub Issues
- 鱼香ROS社区: http://fishros.org.cn

---

### 第6步: 还不行?

- 重启终端
- 重新编译
- 重启电脑(别笑,有时候真管用)

---

## 14.10 预防性措施

与其出了问题再解决,不如提前预防:

---

### 措施1: 养成好习惯

```bash
# 编译后必 source
colcon build && source install/setup.bash

# 或者用这个别名(加到 .bashrc)
alias cb='cd ~/ros2_ws && colcon build && source install/setup.bash'
```

---

### 措施2: 使用版本控制

```bash
cd ~/ros2_ws
git init
git add .
git commit -m "初始版本"

# 每次改完:
git add .
git commit -m "添加了 xxx 功能"

# 搞砸了可以回退
git log
git reset --hard 版本号
```

---

### 措施3: 虚拟机快照

关键节点拍快照:

- 系统装好后
- ROS2 装好后
- 第一个程序跑通后

---

### 措施4: 写测试

```python
# 简单测试
def test_my_function():
    result = my_function(1, 2)
    assert result == 3, "计算错误"
```

---

### 措施5: 模块化开发

不要把所有代码塞一个文件里,拆成多个节点。

好处:

- 容易调试(问题隔离)
- 容易复用
- 出问题影响小

---

## 本章小结

这一章我们梳理了学习 ROS2 过程中最常见的问题:

- ✅ 环境配置(最高频)
- ✅ 编译问题
- ✅ 运行问题
- ✅ 调试技巧
- ✅ 问题排查流程

!!! success "记住这句话"
    **遇到问题不要慌,99% 的问题都有人遇到过。**
    
    善用搜索引擎,善用社区,大部分问题几分钟就能解决。

---

## 实用建议

1. **把常用命令记下来**  
   可以打印这一章的速查表,贴在电脑旁边

2. **遇到新问题记录下来**  
   下次遇到直接翻笔记,比重新搜索快

3. **加入社区**  
   - ROS Discourse  
   - ROS QQ群/微信群  
   - 鱼香ROS社区

4. **帮助别人**  
   教是最好的学。在论坛上回答别人的问题,你会学得更深

---

## 结束

后期我们会推荐一些学习资源,  
帮你在 ROS2 的道路上走得更远!







**联系方式**

- QQ号: 3549289002

- 微信: cyj200310060
