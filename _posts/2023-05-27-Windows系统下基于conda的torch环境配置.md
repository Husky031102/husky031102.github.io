---
title: 【环境配置】Windows系统下基于Conda的Torch环境配置
date: 2023-05-27 00:00:00 +0800
img_path: /assets/img/md
categories:
  - 记录
  - 环境搭建
tags:
  - 记录
  - 环境搭建
---
## Conda配置
看过我上一篇教程的应该对Conda不陌生了吧，我们在写Python的时候可以利用它轻松地做到环境配置和管理，能让我们专心于程序的编写与算法的实现，配合命令行的使用几乎可以做到一键安装，起飞~  
首先需要下载安装Miniconda，[点此下载](https://repo.anaconda.com/miniconda/Miniconda3-latest-Windows-x86_64.exe)，有Anaconda的可以忽略，直接使用Anaconda提供的命令行操作即可  
在系统命令行中输入`conda`，如果出现大量相关信息则说明安装完成  
如果安装过程中没有添加环境变量，则需要手动添加`C:\Users\用户\miniconda3\Scripts`至环境变量并**重启电脑**，请将路径替换为你自己对应的安装位置  
更新Conda  
```shell
conda update conda
```
初始化Conda  
```shell
conda init
```
重启命令行  
如果遇到以下报错  
```text
无法加载文件……profile.ps1
```
以管理员权限运行PowerShell，执行  
```shell
Set-ExecutionPolicy -ExecutionPolicy RemoteSigned
```
输入y，回车  
## 替换镜像源
由于`conda`和`pip`一样默认调用的是国外的下载源，这里我们要进行一个镜像源的替  
执行  
```shell
conda config --set show_channel_urls yes
```

然后在`C:\Users\用户`目录下可以找到`.condarc`文件  
使用记事本打开，替换其中内容为  
```text
channels:
  - defaults
show_channel_urls: true
default_channels:
  - https://mirrors.bfsu.edu.cn/anaconda/pkgs/main
  - https://mirrors.bfsu.edu.cn/anaconda/pkgs/r
  - https://mirrors.bfsu.edu.cn/anaconda/pkgs/msys2
custom_channels:
  conda-forge: https://mirrors.bfsu.edu.cn/anaconda/cloud
  msys2: https://mirrors.bfsu.edu.cn/anaconda/cloud
  bioconda: https://mirrors.bfsu.edu.cn/anaconda/cloud
  menpo: https://mirrors.bfsu.edu.cn/anaconda/cloud
  pytorch: https://mirrors.bfsu.edu.cn/anaconda/cloud
  pytorch-lts: https://mirrors.bfsu.edu.cn/anaconda/cloud
  simpleitk: https://mirrors.bfsu.edu.cn/anaconda/cloud
```
完成镜像源替换后，执行以下指令清空源缓存  
```shell
conda clean -i
```
可通过以下指令确定是否完成换源操作  
```shell
conda config --show
```

## 安装Torch
这里我们采用pip包管理器来安装PyTorch，首先仍然是镜像源的替换  
在用户文件夹下，即`C:\Users\用户`，新建`pip`文件夹，在其中新建`pip.ini`文件，添加以下内容  
```text
[global]
index-url = http://mirrors.aliyun.com/pypi/simple/
[install]
trusted-host = mirrors.aliyun.com
```
在你需要安装的Conda环境中执行以下代码，等待下载完成即可  
```shell
pip3 --default-timeout=9999 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
```
该版本对应的是11.8版本的PyTorch，一键搞定，PyTorch整合了对应版本的CUDA与CUDNN  
## 检查环境
运行以下代码  
```python
import torch
torch.cuda.is_available()
a = torch.rand((1,1)).cuda()
print(a)
print(torch.__version__)
print(torch.version.cuda)
print(torch.backends.cudnn.version())
```

## 附录1
如果不希望每次启动终端都会自动进入base环境，在终端执行以下指令，但是使用深度学习环境时仍然要手动启动嗷  
```shell
conda config --set auto_activate_base false
```
启动默认环境的指令如下  
```shell
conda activate base
```
## 附录2
如果希望自己新建一个独立的环境，执行以下指令，其中，`your_env_name`替换为你想为虚拟环境替换的名字，`x.x`改为你需要的Python版本  
```shell
conda create -n your_env_name python=x.x
```
此后进入该环境只需要输入  
```shell
conda activate your_env_name
```
## 参考网址
[Start Locally | PyTorch](https://pytorch.org/get-started/locally/)  
[Miniconda — conda documentation](https://docs.conda.io/en/latest/miniconda.html)  
[库达快速入门 (nvidia.com)](https://docs.nvidia.com/cuda/cuda-quick-start-guide/index.html#)  