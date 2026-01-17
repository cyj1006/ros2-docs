# 第九章：ROS2 安装教程——安装核心软件

系统准备好了，现在来装 ROS2 Humble。

这里我要 **强烈推荐一个神器**：

!!! success "鱼香 ROS 一键安装脚本"
    **鱼香 ROS** 的一键安装脚本。
    
    这是 B 站 up 主 **"鱼香 ROS"（小鱼）** 开发的，
    
    在国内 ROS 圈子里非常出名。

---

## 9.1 为什么推荐鱼香 ROS 一键安装

先说说官方安装方法的痛点：

---

### 官方安装的问题

!!! warning "复杂繁琐"
    1. **要手动添加 GPG 密钥**
       - 命令贼长，容易敲错
    
    2. **要配置软件源**
       - 国外源下载慢得要死
       - 经常超时失败
    
    3. **要装一堆依赖**
       - 漏装一个后面就报错
    
    4. **环境变量要自己配**
       - 新手容易忘

---

### 鱼香 ROS 的优势

!!! success "一键搞定"
    - ✓ **自动识别** 你的 Ubuntu 版本
    - ✓ **自动换** 国内源（下载飞快）
    - ✓ **自动安装** ROS2 和所有依赖
    - ✓ **自动配置** 环境变量
    - ✓ **全程中文提示**，傻瓜式操作

---

### 我的真实体验

!!! quote "血泪教训"
    说实话，我带的好几届学弟学妹，
    
    用官方方法装的 **十个有八个出问题**，
    
    用鱼香 ROS 装的 **基本一遍过**。

---

## 9.2 一键安装 ROS2

### 整个安装过程就一行命令

```bash
wget http://fishros.com/install -O fishros && . fishros
```

!!! tip "操作步骤"
    1. 打开终端
    2. 复制这行命令
    3. 粘贴到终端里（`Ctrl+Shift+V`）
    4. 回车执行

---

### 等待下载

脚本会下载并运行，然后出现一个菜单。

---

## 9.3 安装步骤详解

### 第一步：选择安装 ROS

会出现类似这样的菜单：

```
========== 鱼香ROS一键安装 ==========
[1]:一键安装:ROS(支持ROS和ROS2,树莓派Jetson)
[2]:一键安装:github桌面版(gitDesktop)
[3]:一键安装:nodejs
...
请选择:
```

!!! info "选择"
    输入 `1` 然后回车，选择一键安装 ROS

---

### 第二步：是否换源

```
[1]:更换系统源再继续安装
[2]:不更换继续安装
请选择:
```

!!! success "必须换源"
    输入 `1`，换源！
    
    这步很重要，换成国内源下载速度能快 **十倍不止**。

---

### 第三步：清理第三方源

```
[1]:仅更换系统源
[2]:更换系统源并清理第三方源
请选择:
```

!!! info "建议"
    - 如果你是 **新装的系统**，选 `2` 比较干净
    - 如果之前折腾过怕影响其他软件，选 `1`

---

### 第四步：选择 ROS 版本

```
[1]:humble(ROS2)
[2]:jazzy(ROS2)
[3]:foxy(ROS2)
[4]:noetic(ROS1)
...
请选择:
```

!!! success "推荐选择"
    我们要装 **ROS2 Humble**，输入 `1`

---

### 第五步：选择安装类型

```
[1]:humble(ROS2)桌面版
[2]:humble(ROS2)基础版(不含图形界面)
请选择:
```

!!! success "必选桌面版"
    选 `1`，桌面版。
    
    桌面版包含 RViz2、Gazebo 这些工具，**学习必备**。

---

### 等待安装

然后就是等待了。

!!! info "安装过程"
    脚本会 **自动下载安装** 所有东西，
    
    全程不用你管。
    
    根据网速不同，大概需要 **10-30 分钟**。

---

### 中间可能的提示

#### rosdep 配置

中间可能会问你要不要配置 rosdep：

```
是否安装rosdep? [y/n]
```

!!! info "建议"
    选 `y`，这个工具用来解决依赖的，建议装。

---

#### 其他问题

如果有其他提示，一般选 **"是"** 就行。

---

## 9.4 安装过程中可能遇到的情况

### 情况1：下载速度很慢怎么办？

!!! tip "解决方法"
    如果换源之后还是慢，可能是你的网络本身就不太行。
    
    可以试试：
    - 用手机热点（有时候反而更快）
    - 换个时间段（晚上人少可能快一些）

---

### 情况2：中途断了怎么办？

!!! success "不用担心"
    重新运行那行命令就行，
    
    脚本会接着装，不会重复下载。

---

### 情况3：报错了怎么办？

!!! info "查找解决方案"
    把报错信息记下来，去以下地方搜一下：
    
    - **B 站鱼香 ROS 的视频评论区**
    - **鱼香 ROS 社区**（fishros.org.cn）
    - **百度/Google 搜索报错信息**
    
    基本都有人遇到过，都有解决方案。

---

## 9.5 验证安装是否成功

安装完成后，**关掉当前终端**，重新打开一个新终端。

!!! danger "重要"
    这步很重要，不然环境变量不生效。

---

### 测试1：检查 ros2 命令

```bash
ros2
```

!!! success "成功标志"
    如果出现 ros2 的帮助信息，说明装好了。

---

### 测试2：经典测试 - talker & listener

开两个终端：

---

#### 终端 1：

```bash
ros2 run demo_nodes_cpp talker
```

你会看到不停地发送 "Hello World"。

---

#### 终端 2：

```bash
ros2 run demo_nodes_cpp listener
```

你会看到不停地接收 "Hello World"。

!!! success "通信成功"
    如果看到一个在发，一个在收，**恭喜你，安装成功！**

---

### 测试3：小海龟（最经典）

#### 终端 1：启动小海龟

```bash
ros2 run turtlesim turtlesim_node
```

!!! info "应该看到"
    会弹出一个窗口，里面有只小海龟。

---

#### 终端 2：键盘控制

```bash
ros2 run turtlesim turtle_teleop_key
```

!!! success "测试成功"
    现在可以用键盘方向键控制小海龟移动了！
    
    试试让它画个圈？

---

## 9.6 创建工作空间

工作空间是放你自己代码的地方。

---

### 创建工作空间

```bash
# 创建工作空间目录
mkdir -p ~/ros2_ws/src

# 进入工作空间
cd ~/ros2_ws

# 编译（初始化工作空间）
colcon build
```

!!! info "第一次编译"
    第一次 build 会初始化工作空间，
    
    虽然 src 目录是空的，但这步不能省。

---

### 配置环境变量

```bash
# 将工作空间的环境添加到 bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc

# 重新加载 bashrc
source ~/.bashrc
```

!!! warning "环境变量说明"
    这样每次打开终端，
    
    工作空间的环境就会自动加载。

---

## 9.7 安装一些常用工具（可选）

### 再次运行鱼香 ROS 脚本

```bash
wget http://fishros.com/install -O fishros && . fishros
```

---

### 菜单里还有其他东西可以装

!!! info "推荐安装"
    - **rosdep 配置**（解决依赖用的，建议装）
    - **Gazebo 仿真器**（如果没自动装）
    - **其他开发工具**

根据需要选择安装就行。

---

## 9.8 再拍个快照

环境搭好了，别忘了给虚拟机拍个快照。

---

### 操作步骤

1. 关闭虚拟机（或挂起）
2. 在 VMware 里：**"虚拟机" → "快照" → "拍摄快照"**
3. 命名为 **"ROS2 安装完成"**
4. 点击 **"拍摄快照"**

!!! success "保险措施"
    万一后面折腾出问题，
    
    可以一键恢复到这个状态。

---

## 9.9 感谢鱼香 ROS

!!! quote "致谢"
    最后说一句，**鱼香 ROS（小鱼）** 为国内 ROS 社区做了很多贡献。
    
    除了这个一键安装脚本，他还有很多免费的 ROS2 教程，
    
    B 站上可以搜 **"鱼香 ROS"** 找到。

!!! success "支持开源"
    如果这个脚本帮到了你，
    
    可以去给他点个关注支持一下。

---

## 9.10 手动安装方法（备选）

如果你因为某些原因不想用一键脚本，

也可以按照官方方法手动安装。

---

### 官方安装步骤（简略版）

```bash
# 1. 设置 locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 2. 添加 ROS2 apt 源
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) \
    signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu \
    $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
    sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 3. 安装 ROS2
sudo apt update
sudo apt upgrade -y
sudo apt install ros-humble-desktop -y

# 4. 配置环境
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 5. 安装其他工具
sudo apt install python3-colcon-common-extensions -y
sudo apt install python3-rosdep python3-rosinstall \
    python3-rosinstall-generator python3-wstool build-essential -y

# 6. 初始化 rosdep
sudo rosdep init
rosdep update
```

!!! warning "不推荐新手"
    这个方法步骤多，容易出错，
    
    **不推荐新手使用**。

---

## 9.11 常见问题

### 问题1：找不到 ros2 命令

!!! info "解决方法"
    ```bash
    # 手动 source 一下
    source /opt/ros/humble/setup.bash
    
    # 检查 .bashrc 里有没有这行
    cat ~/.bashrc | grep ros
    
    # 如果没有，手动添加
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    ```

---

### 问题2：colcon build 报错

!!! info "解决方法"
    ```bash
    # 清除缓存重新编译
    cd ~/ros2_ws
    rm -rf build install log
    colcon build
    ```

---

### 问题3：小海龟窗口打不开

!!! info "可能原因"
    1. **没装桌面版**
       - 重新安装，选桌面版
    
    2. **环境变量没加载**
       - `source ~/.bashrc`
    
    3. **VMware 3D 加速没开**
       - 检查虚拟机设置

---

## 9.12 环境变量说明

### ROS2 的环境变量

!!! info "工作原理"
    每次打开终端，需要加载 ROS2 的环境：
    
    ```bash
    source /opt/ros/humble/setup.bash
    ```
    
    这个命令会设置一堆环境变量，
    
    让系统知道去哪里找 ROS2 的命令和包。

---

### 自动加载

!!! success "一劳永逸"
    我们把这行加到 `~/.bashrc`，
    
    就不用每次手动 source 了。
    
    ```bash
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    ```

---

### 工作空间的环境

如果你有自己的工作空间：

```bash
source ~/ros2_ws/install/setup.bash
```

也要加到 `.bashrc`。

!!! warning "顺序重要"
    先 source ROS2 的环境，
    
    再 source 工作空间的环境。

---

## 9.13 验证安装的完整性

### 检查安装的包

```bash
# 列出所有 ROS2 的包
ros2 pkg list

# 应该能看到很多包，比如：
# - turtlesim
# - demo_nodes_cpp
# - demo_nodes_py
# - ...
```

---

### 检查工具

```bash
# 检查 colcon
colcon version

# 检查 RViz2
rviz2 --version

# 检查 Gazebo
gazebo --version
```

!!! success "全部有输出"
    如果这些命令都有正常输出，
    
    说明安装很完整。

---

## 本章小结

通过这一章，你应该已经：

1. **成功安装了 ROS2 Humble**

    使用鱼香 ROS 一键脚本

    或者使用官方方法手动安装

2. **验证了安装**

    talker & listener 测试

    小海龟测试

3. **创建了工作空间**

    ~/ros2_ws

4. **配置了环境变量**

    自动加载 ROS2 环境

---

## 下一章预告

下一章我们会进入 **核心知识**：

> **ROS2 核心概念详解——最重要的知识点**

详细讲解节点、话题、服务、动作等核心概念，

这是学好 ROS2 的基础。

