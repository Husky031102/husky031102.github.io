---
title: 利用wsl完成Ubuntu下深度学习环境安装
date: 2023-05-07 00:00:00 +0800
img_path: /assets/img/md
categories: [记录, 环境搭建]
tags: [记录，环境搭建]     # TAG names should always be lowercase
---
按照常规方案，我们首先安装ubuntu双系统，依照以下教程可自行完成  
[Windows 和 Ubuntu 双系统的安装和卸载](https://www.bilibili.com/video/BV1554y1n7zv/?share_source=copy_web&vd_source=b70d2d828fe907ea790f92edff598b54)  
基于ubuntu的环境配置较为简易，读者可自行寻找教程安装  
考虑到双系统切换起来可能造成一定麻烦，同时Linux系统本身不适合日常使用与娱乐，在轻度使用条件下较为不便，而普通虚拟机虽然方便安全，在性能上却损失较多，笔者本次尝试使用wsl2来配置ubuntu环境，其优势在于可以通过vscode进行便捷的访问操控linux文件，还能够有效的协同windows文件  
虽说wsl2本质上是一个虚拟机的性质，但是其性能损耗几乎可以无视，对日常使用而言已经足够，开个玩笑说windows正在向最好的linux发行版迈进也不为过  
其安装过程较为简易，但是对C盘空间需求较大，请读者提前注意，其本质上可以看做对系统功能的一个拓展，所以尝试将其移动到其他位置可能造成各种奇怪的影响，为方便期间本文按照默认操作进行，总共花费约50G+空间，如果系统盘空间紧缺可在安装完成系统后参照下图自行尝试，安装系统约莫需要10G空间  
![移动wsl](移动wsl.png)
## 安装wsl
首先使用管理员权限打开powershell，具体方法为，按住Win+R键打开运行窗口，输入powershell，在任务栏右键单击powershell窗口选择以管理员身份运行  
在管理员权限下输入 `wsl --install`后回车即可等待自动完成安装，其默认安装版本即为wsl2  
ubuntu系统需要设置用户名和密码，请注意用户名**只能**为小写的英文或数字，密码在使用sudo命令时需要频繁输入，建议越短越好，在输入密码时密码是**不可见**的，请提前注意  
安装完成后需要重启，具体请依照提示进行  
安装过程中如若遇见错误信息请复制错误信息到搜索引擎尝试解决  
安装完毕后可参考下列官方文档进行相关配置操作，此处笔者按简易流程快速配置  
[设置 WSL 开发环境 | Microsoft Learn](https://learn.microsoft.com/zh-cn/windows/wsl/setup/environment)  
重启后再次重复以上步骤打开powershell，输入`wsl`即可进入ubuntu系统，输入`cd`切换到标准工作目录，这个操作以后很经常要使用的  
输入`wsl --update`来更新wsl内核  
输入`sudo apt update && sudo apt upgrade`等待依赖软件自动完成更新  
考虑到python各个库之间的依赖关系极为复杂，我们采取conda来进行环境管理，为节约空间，我们使用只保留了基本功能的miniconda来进行管理  
## 安装miniconda
大家应该是使用过anaconda的，这是一个整合了大部分常用科研库的库管理环境，但是其大小不是很友好，我们遵循用啥下啥的原则，删改了大部分乱七八糟的东西，比如说大家喜欢的spyder编辑器这种，只留下最核心的包管理功能，来管理我们的环境  
输入`cd`回到标准工作目录  
输入`wget https://mirrors.ustc.edu.cn/anaconda/miniconda/Miniconda3-latest-Linux-x86_64.sh`   从镜像源下载miniconda  
输入`bash  Miniconda3-latest-Linux-x86_64.sh`安装  
此时会有一段协议让你阅读，快速拉下来然后输入`yes`即可开始安装  
完成安装后可以看见一个感谢信~此时一定要**重启终端**  
重启后输入`conda --v`检查是否成功安装，如果有反馈值则说明安装成功  
## 配置pytorch环境
### cuda安装
cuda可以简单理解成一个用于调用gpu加速的东西，能够大大提升我们深度学习的训练效率，比如拿AI画图吃显卡就是依靠cuda进行的  
在命令窗口中输入`nvidia-smi`即可查看自身显卡信息，查看能够适配的cuda最高版本，这个命令是用于调度硬件信息的，可以用于ubuntu系统终端也可以用于powershell  
cuda安装可参考以下NVIDIA的官方文档  
[WSL 2 上的 NVIDIA GPU 加速计算](https://docs.nvidia.com/cuda/wsl-user-guide/index.html)  
首先依旧是打开wsl终端，输入`cd`切换到工作目录  
输入`sudo apt-key del 7fa2af80`清除旧的key  
依次输入以下命令完成cuda安装  
```shell
wget https://developer.download.nvidia.com/compute/cuda/repos/wsl-ubuntu/x86_64/cuda-wsl-ubuntu.pin
sudo mv cuda-wsl-ubuntu.pin /etc/apt/preferences.d/cuda-repository-pin-600
wget https://developer.download.nvidia.com/compute/cuda/12.1.1/local_installers/cuda-repo-wsl-ubuntu-12-1-local_12.1.1-1_amd64.deb
sudo dpkg -i cuda-repo-wsl-ubuntu-12-1-local_12.1.1-1_amd64.deb
sudo cp /var/cuda-repo-wsl-ubuntu-12-1-local/cuda-*-keyring.gpg /usr/share/keyrings/
sudo apt-get update
sudo apt-get -y install cuda
```
其实按这个步骤安装是需要一定前置条件的，即windows系统本身具有NVIDIA驱动，不过常用独显笔记本应该都是出厂自带的，所以此处不做强调，请不要再在wsl中额外下载其余驱动以免产生覆盖
安装速度较慢，建议发现下载时长较长时睡一觉或者做些别的事情  
### conda环境设置
使用conda是为了创造出一个独立的环境，方便对环境进行管理，因此我们先创造一个自己使用的环境  
在wsl终端输入`cd`切换到工作目录，输入`conda create --name d2l python=3.9 -y`创造一个python版本3.9的环境，此处d2l可以替换成任意自己喜欢的名字，换成笔者首字母缩写也不是不行~  
输入`conda activate d2l`激活名字为d2l的环境，此时可以看见终端命令行的最前方变成了(d2l)，此后进行的pip或者conda下载的库都会只在d2l环境中生效  
此后每次使用**都需要**输入`conda activate d2l`来进入我们设定的环境  
### 下载pytorch
使用`pip install torch torchvision`即可自动下载所有附带依赖库  
完成下载后输入`python`进入python终端  
在python终端输入  
```python
import torch  
torch.cuda.is_available()
a = torch.rand((1,1)).cuda()
print(a)
```
如果正常输出结果，则完成环境配置  
如果遇到pip下载缓慢的情况，可以切换pip镜像源，然后重新输入pip命令  
在~/.pip/pip.conf中添加以下内容  
```text
[global]
index-url = https://mirrors.aliyun.com/pypi/simple/

[install]
trusted-host=mirrors.aliyun.com

```
~即为在终端`cd`后的工作目录，如果没有该文件夹和文件，则自行创立一个就好，大家可能对纯命令行环境怎么打开目录修改文件不是很熟悉，没事哥们其实也不怎么熟练，为方便在wsl系统中进行文本编辑，我们来使用vscode对wsl系统中的文件进行编辑  
### vscode配置
vscode是一个很棒的免费的文本编辑器，笔者自己也直在用的，可以通过各种花里胡哨的插件来打造适合自己的编程体验，这里留给大家自己去体验自己探索，本文主要研究怎么通过vsc编辑wsl中的文件  
在windows系统中下载vscode，打开后会自动识别wsl环境并且安装相关插件，我们打开左侧功能栏最下方的电脑样式的图标，也就是远程资源管理器，双击ubuntu后，vsc会自动进行相关配置，完成后即可通过上方搜索栏轻松的访问wsl系统目录，编辑wsl中的文件，这真是太酷了（bushi  
相对于命令行的wsl，vsc的图像化界面还是可以自己慢慢研究慢慢玩的，这里就不多说了  
以上就是环境配置的大致流程，感谢观看至此，希望读者能够顺利完成环境配置工作  
## windows Terminal 
你可能会觉得反复重复打开powershell再启动wsl的过程很麻烦，那你大可以试试微软整合的终端，这是一个整合了powershell，cmd，ubuntu等各类终端的超级终端，可以不再一遍遍的Win+R了，好耶  
微软文档如下  
[视窗终端安装 |微软学习 (microsoft.com)](https://learn.microsoft.com/en-us/windows/terminal/install)  
考虑到app store本身打开是很玄学的一件事情，笔者更推荐从github下载,下载链接如下，解压后安装即可正常使用  
[点此下载](https://github.com/microsoft/terminal/releases/download/v1.16.10261.0/Microsoft.WindowsTerminal_Win10_1.16.10261.0_8wekyb3d8bbwe.msixbundle_Windows10_PreinstallKit.zip)  
具体使用方式可自行阅读文档或参考终端的设置界面  
在开始菜单即可找到安装的终端程序，推荐将其固定至底部任务栏方便启用  
# Q&A
遇到任何问题欢迎找笔者深入讨论研究，笔者再对本文做修正补充，于此处添加修正方案  
1. 修正了关于`nvidia-smi`的错误理解，事实上该命令仅用于调度显卡硬件信息，其显示的cuda表示本机显卡所支持的最新cuda版本，并非已安装cuda的版本，要检验是否成功安装仍然需要在最后运行python代码实际检验，给出验证代码一例如下  
```python
import torch
print(torch.__version__)
print(torch.version.cuda)
print(torch.backends.cudnn.version())
```


