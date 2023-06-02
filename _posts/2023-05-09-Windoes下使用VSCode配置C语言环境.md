---
title: Windoes下使用VSCode配置C语言环境
date: 2023-05-09 00:00:00 +0800
img_path: /assets/img/md
categories: [记录, 环境搭建]
tags: [记录，环境搭建]     # TAG names should always be lowercase
---
重装完系统后才发现原来已经折腾过那么多的环境了，重走一遍的时候还是比第一次要顺畅的多的，这大概也是一种成长吧  
闲话少说，直接步入正题，还是打开我们熟悉的vsc,然后在插件市场搜索C，安装微软官方提供的C/C++插件，也可以自己再选配语法提示插件与美化插件等，这个可以自己再去网上找找相关资料看看大伙儿推荐的宝藏插件  
为了方便起见，我们不直接去亲自安装gcc等工具链，而是效仿linux的办法用一个统一的包管理器去轻松的完成配置，也就是我们要用到的MSYS2，下载的话可以直接[单击此处](https://github.com/msys2/msys2-installer/releases/download/2022-06-03/msys2-x86_64-20220603.exe)  
一路点点点完成安装后，最后一步我们先**不勾选**直接运行MSYS2，因为其默认的镜像源都在国外，下载速度一言难尽，我们先修改个镜像源  
分别打开`安装后的主目录\etc\pacman.d`，对该目录下的`mirrorlist.msys`,  
使用VSCode打开，在第一个`Server = xxx `前添加  
`Server = https://sourceforge.net/projects/msys2/files/REPOS/MSYS2/$arch/`  
保存退出  
对`mirrorlist.mingw32`文件，操作一致，添加  
`Server = https://sourceforge.net/projects/msys2/files/REPOS/MINGW/i686/`  
对`mirrorlist.mingw64`文件，添加  
`Server = https://sourceforge.net/projects/msys2/files/REPOS/MINGW/x86_64/`  
完成以上操作后，我们再打开`msys2.exe`  
惯例首先进行一次更新，输入  
`pacman -Syu`  
然后一路yes等待资源更新完成  
输入  
`pacman -S --needed base-devel mingw-w64-x86_64-toolchain`  
完成安装后，我们还需要将工具链加入环境变量，由于我们采用的是MSYS2的集中管理，所以直接添加MSYS2到环境变量即可  
在左下角的搜索栏搜索编辑系统环境变量，打开后点击右下角的环境变量，我们这里只修改当前用户的环境变量，双击位于上半窗口的`path`，点击新建，输入`安装后的主目录\mingw64\bin`，点击两个确定后**重启计算机**  
重启后打开powershell，输入  
```text
gcc --version
g++ --version
gdb --version
```
如果命令行反馈正常则说明安装完成  

参考资料如下  
[Get Started with C++ and Mingw-w64 in Visual Studio Code](https://code.visualstudio.com/docs/cpp/config-mingw)  
[(31条消息) MSYS2使用教程——win10系统64位安装msys2最新版（msys2-x86_xxxx.exe）_speedhack-x86_64_Dreamhai的博客-CSDN博客](https://blog.csdn.net/Dreamhai/article/details/109842184)  

