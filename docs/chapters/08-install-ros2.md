# 第八章：Ubuntu 系统安装教程——搭建环境

这一章手把手教你安装 Ubuntu 22.04 系统。

我们采用 **虚拟机** 的方式，这是最安全的方式，新手照着做就行。

---

## 8.1 准备工作

在开始之前，需要下载两样东西：

---

### 8.1.1 下载 VMware Workstation Player

#### Windows 用户

!!! info "下载链接"
    - 官网：https://www.vmware.com/products/workstation-player.html
    - 选择免费版（Player）即可

---

#### Mac 用户

!!! warning "注意"
    Mac 用户需要下载 **VMware Fusion**
    
    或者使用 **Parallels Desktop**（更好用但需要付费）

---

#### 安装 VMware

1. 下载安装包
2. 双击运行
3. 一路下一步
4. 选择"个人使用"（免费）

!!! tip "安装提示"
    安装过程很简单，基本都是默认选项。

---

### 8.1.2 下载 Ubuntu 22.04 ISO 镜像

#### 官方下载

!!! info "官方网站"
    https://ubuntu.com/download/desktop

选择 **Ubuntu 22.04 LTS**

---

#### 国内镜像（推荐，速度快）

!!! success "推荐下载源"
    - **清华大学镜像站**
      - https://mirrors.tuna.tsinghua.edu.cn/ubuntu-releases/22.04/
    
    - **阿里云镜像站**
      - https://mirrors.aliyun.com/ubuntu-releases/22.04/
    
    - **中科大镜像站**
      - https://mirrors.ustc.edu.cn/ubuntu-releases/22.04/

---

#### 选择正确的文件

!!! warning "注意选择"
    下载文件名类似：
    
    `ubuntu-22.04.x-desktop-amd64.iso`
    
    - **desktop**：桌面版（我们要的）
    - **amd64**：64 位系统（大部分电脑）

---

### 8.1.3 检查硬件要求

确保你的电脑满足以下要求：

| 项目 | 最低要求 | 推荐配置 |
|------|---------|---------|
| **CPU** | 双核 | 四核及以上 |
| **内存** | 4GB | 8GB 及以上 |
| **硬盘空间** | 50GB | 100GB 及以上 |

!!! tip "硬盘空间说明"
    这里说的是给虚拟机分配的空间，
    
    你的实际硬盘要更大一些。

---

## 8.2 创建虚拟机

### 8.2.1 启动 VMware

打开 VMware Workstation Player

点击 **"创建新虚拟机"**

---

### 8.2.2 选择安装方式

!!! info "安装源选择"
    选择 **"安装程序光盘映像文件(iso)"**
    
    点击 **"浏览"**，选择刚才下载的 Ubuntu ISO 文件

---

### 8.2.3 配置用户信息

VMware 会自动检测到 Ubuntu 安装程序，要求你输入：

!!! warning "重要提醒"
    - **全名**：可以是中文
    - **用户名**：**只能用英文小写，不能有空格**
    - **密码**：自己设一个，别忘了

**示例：**

```
全名：张三
用户名：zhangsan
密码：********
```

!!! danger "常见错误"
    用户名不能用中文、大写字母、空格！
    
    比如：`Zhang San`、`ZhangSan`、`张三` 都是错的

---

### 8.2.4 配置虚拟机位置

!!! info "存储位置"
    - **虚拟机名称**：Ubuntu 22.04
    - **位置**：选择一个空间 最好不要放在 C 盘（Windows）

---

### 8.2.5 配置磁盘大小

!!! success "推荐设置"
    - **最大磁盘大小**：100GB
    - **存储方式**：选择 "将虚拟磁盘拆分成多个文件"

!!! info "说明"
    虚拟机使用的是 **动态分配**，
    
    不会立刻占用 100GB，
    
    而是用多少占多少。

---

### 8.2.6 自定义硬件

点击 **"自定义硬件"**，进行以下调整：

---

#### 内存

!!! success "推荐设置"
    - 最少：4GB
    - 推荐：8GB
    - 如果你电脑内存够大：16GB

---

#### 处理器

!!! success "推荐设置"
    - **处理器数量**：1
    - **每个处理器的核心数量**：4
    
    （总共 4 核）

!!! tip "核心分配建议"
    给虚拟机分配你 CPU 的一半核心数。
    
    比如你的 CPU 是 8 核，就给虚拟机 4 核。

---

#### 显示器（重要！）

!!! danger "必须设置"
    **一定要勾选 "加速 3D 图形"**
    
    否则 Gazebo 会黑屏！

!!! success "推荐设置"
    - ☑ 加速 3D 图形
    - 图形内存：2GB 或更多

---

#### 网络适配器

!!! info "默认设置"
    保持默认的 **NAT 模式** 即可

---

#### 其他设备

!!! info "可选"
    - 可以移除打印机、软盘等用不到的设备
    - 保持 CD/DVD 连接 ISO 镜像

---

### 8.2.7 完成创建

点击 **"完成"**

虚拟机创建完成，但还没有启动。

---

## 8.3 安装 Ubuntu

### 8.3.1 启动虚拟机

在 VMware 主界面，点击 **"开启此虚拟机"**

---

### 8.3.2 等待自动安装

!!! info "自动化安装"
    由于我们使用了 VMware 的简易安装功能，
    
    Ubuntu 会 **自动安装**。
    
    整个过程大约 10-20 分钟，不需要你操作。

---

#### 你会看到：

1. Ubuntu 安装界面
2. 自动选择语言、键盘
3. 自动分区
4. 自动安装软件
5. 自动重启

!!! tip "耐心等待"
    这期间可以去喝杯水，休息一下。

---

### 8.3.3 首次登录

安装完成后，虚拟机会自动重启。

!!! success "登录"
    输入你之前设置的密码，登录系统。

---

### 8.3.4 初始配置

首次登录会有一些引导界面：

1. **在线账号**：可以跳过
2. **Livepatch**：可以跳过
3. **帮助改进 Ubuntu**：选择"否"
4. **隐私设置**：根据个人喜好

!!! tip "快速设置"
    这些都可以跳过，不影响使用。

---

## 8.4 安装后的配置

### 8.4.1 安装 VMware Tools

VMware Tools 可以提供：

- 更好的显示效果
- 共享文件夹
- 复制粘贴功能

---

#### 安装步骤

!!! info "通常会自动安装"
    现代版本的 Ubuntu 和 VMware 通常会自动安装 VMware Tools。
    
    如果没有自动安装：

1. 在 VMware 菜单选择 **"虚拟机" → "安装 VMware Tools"**
2. Ubuntu 桌面会出现一个光盘图标
3. 打开终端，运行：

```bash
cd /media/你的用户名/VMware\ Tools
tar xzvf VMwareTools-*.tar.gz -C /tmp
cd /tmp/vmware-tools-distrib
sudo ./vmware-install.pl
```

4. 一路回车默认选项
5. 重启虚拟机

---

### 8.4.2 更换软件源（重要！）

Ubuntu 默认使用国外源，下载速度 **非常慢**。

!!! success "必须换成国内源"
    换成国内源后，下载速度能快 **10 倍以上**。

---

#### 方法一：图形化界面

1. 打开 **"软件和更新"**（Software & Updates）
   - 可以在应用程序菜单里搜索

2. 在 **"下载自"** 那里点击下拉菜单

3. 选择 **"其他站点"**

4. 点击 **"选择最佳服务器"**
   - 或者手动选择国内镜像：
     - 阿里云
     - 清华大学
     - 中科大

5. 点击 **"选择服务器"**

6. 提示输入密码，输入你的密码

7. 点击 **"关闭"**，会提示更新软件列二：命令行（推荐，更快）

```bash
# 备份原来的源
sudo cp /etc/apt/sources.list /etc/apt/sources.list.backup

# 修改源文件
sudo nano /etc/apt/sources.list
```

删除所有内容，替换为以下内容（阿里云镜像）：

```
deb http://mirrors.aliyun.com/ubuntu/ jammy main restricted universe multiverse
deb http://mirrors.aliyun.com/ubuntu/ jammy-security main restricted universe multiverse
deb http://mirrors.aliyun.com/ubuntu/ jammy-updates main restricted universe multiverse
deb http://mirrors.aliyun.com/ubuntu/ jammy-backports main restricted universe multiverse
```

保存退出：`Ctrl+O` → 回车 → `Ctrl+X`

---

### 8.4.3 更新系统

换好源后，立刻更新系统：

```bash
# 更新软件包列表
sudo apt update

# 升级已安装的软件
sudo apt upgrade -y

# 清理不需要的包
sudo apt autoremove -y
```

!!! info "更新时间"
    第一次更新可能需要 5-10 分钟，
    
    取决于网速和系统更新的内容。

---

### 8.4.4 安装常用工具

```bash
sudo apt install -y \
    git \
    vim \
    curl \
    wget \
    terminator \
    tree \
    net-tools \
    htop
```

!!! info "工具说明"
    - **git**：版本控制
    - **vim**：文本编辑器
    - **curl/wget**：下载工具
    - **terminator**：好用的终端
    - **tree**：目录树显示
    - **net-tools**：网络工具
    - **htop**：系统监控

---

### 8.4.5 安装中文输入法（可选）

如果需要在 Ubuntu 里输入中文：

```bash
# 安装 ibus 拼音输入法
sudo apt install -y ibus-pinyin

# 重启 ibus
ibus restart

# 或者安装搜狗输入法（需要额外步骤）
```

---

### 8.4.6 配置 SSH（可选）

如果想从 Windows 通过 SSH 连接虚拟机：

```bash
# 安装 SSH 服务器
sudo apt install -y openssh-server

# 启动 SSH 服务
sudo systemctl start ssh

# 设置开机自启
sudo systemctl enable ssh

# 查看 IP 地址
ip addr show
```

然后在 Windows 可以用 **PuTTY** 或 **MobaXterm** 连接。

---

## 8.5 虚拟机使用技巧

### 8.5.1 快照功能（非常重要！）

!!! success "强烈建议"
    在装 ROS2 之前，给虚拟机拍个快照。
    
    万一后面折腾坏了，可以一键恢复。

---

#### 创建快照

1. 确保虚拟机已关机或挂起
2. 在 VMware 菜单选择 **"虚拟机" → "快照" → "拍摄快照"**
3. 输入快照名称，如 **"系统安装完成"**
4. 点击 **"拍摄快照"**

---

#### 恢复快照

如果系统出问题了：

1. 关闭虚拟机
2. **"虚拟机" → "快照" → "快照管理器"**
3. 选择要恢复的快照
4. 点击 **"恢复"**

!!! tip "快照建议"
    建议在以下时间点拍快照：
    
    - 系统安装完成
    - ROS2 安装完成
    - 重要项目开始前

---

### 8.5.2 共享文件夹

可以在 Windows 和 Ubuntu 之间共享文件。

---

#### 设置方法

1. **"虚拟机" → "设置" → "选项" → "共享文件夹"**
2. 选择 **"总是启用"**
3. 点击 **"添加"**
4. 选择 Windows 上的文件夹
5. 给它起个名字

---

#### 访问共享文件夹

在 Ubuntu 里：

```bash
cd /mnt/hgfs/共享文件夹名称
```

---

### 8.5.3 全屏显示

!!! tip "快捷键"
    - **进入全屏**：`Ctrl+Alt+Enter`
    - **退出全屏**：`Ctrl+Alt`

---

### 8.5.4 复制粘贴

VMware Tools 安装后，可以在 Windows 和 Ubuntu 之间直接复制粘贴文本。

!!! info "如果不行"
    重启虚拟机通常能解决问题。

---

## 8.6 常见问题

### 问题1：虚拟机启动很慢

!!! tip "解决方法"
    - 确保分配了足够的内存（8GB）
    - 确保 SSD 有足够空间
    - 关闭不必要的后台程序

---

### 问题2：Gazebo 黑屏

!!! warning "最常见问题"
    **原因：** 没有开启 3D 加速

**解决方法：**

1. 关闭虚拟机
2. 编辑虚拟机设置
3. 显示器 → 勾选 "加速 3D 图形"
4. 重启虚拟机

---

### 问题3：网络连接不上

!!! info "检查步骤"
    1. 检查虚拟机设置 → 网络适配器 → 确保是 NAT 模式
    2. 在 Ubuntu 里运行：
       ```bash
       ip addr show
       ```
    3. 如果没有 IP，重启虚拟机

---

### 问题4：分辨率调整不了

!!! tip "解决方法"
    1. 确保安装了 VMware Tools
    2. 重启虚拟机
    3. 在 Ubuntu 设置 → 显示里调整

---

## 8.7 性能优化建议

### 8.7.1 关闭不必要的后台服务

```bash
# 禁用自动更新（可选）
sudo systemctl disable unattended-upgrades
```

---

### 8.7.2 调整虚拟机配置

!!! info "如果电脑性能不够"
    可以适当降低：
    - 内存：降到 6GB 或 4GB
    - CPU 核心：降到 2 核
    - 3D 图形内存：降到 1GB

---

### 8.7.3 使用 SSD

!!! success "最有效的优化"
    如果可能，把虚拟机文件放在 SSD 上，
    
    速度会快很多。

---

## 本章小结

通过这一章，你应该已经：

1. **成功安装了 Ubuntu 22.04 虚拟机**

    下载了 VMware 和 Ubuntu ISO

    创建并配置了虚拟机
    
    完成了系统安装

2. **完成了基本配置**

    更换了软件源

    更新了系统

    安装了常用工具

3. **学会了虚拟机的使用**

    创建快照

    共享文件夹

    常见问题处理

---

## 下一章预告

下一章我们会进入 **最激动人心的部分**：

> **ROS2 安装教程——安装核心软件**

我会推荐使用 **鱼香 ROS 的一键安装脚本**，

让安装过程变得超级简单。

