---
title: 【环境配置】利用Clion在WSL下进行ROS开发
date: 2023-09-17 00:00:00 +0800
img_path: /assets/img/md
categories:
  - 记录
  - 环境搭建
tags:
  - 记录
  - 环境搭建
version: "2.0"
---
考虑到网上有关的教程只涉及到Clion，WSL与ROS中的两两结合，这里介绍一种适用于Clion开发WSL下的ROS的环境配置方法  
## Clion配置WSL  
此处可以参考官方提供的方法  
[WSL2 | CLion Documentation](https://www.jetbrains.com/help/clion/how-to-use-wsl-development-environment-in-product.html)  
在Clion中打开设置页，选择构建，执行，部署，选择工具链，新建WSL工具链并设为默认工具链  
![2023-09-17-利用Clion在WSL下进行ROS开发-Clion设置WSL工具链](2023-09-17-利用Clion在WSL下进行ROS开发-Clion设置WSL工具链.png)  
如果你选择使用默认的CMake，那么正常的使用生成，运行和调试，享受现代化IDE的快乐吧  
## Clion配置WSL下ROS开发环境
### 环境配置
参考以下  
[小鱼的一键安装系列 | 鱼香ROS](https://fishros.org.cn/forum/topic/20/%E5%B0%8F%E9%B1%BC%E7%9A%84%E4%B8%80%E9%94%AE%E5%AE%89%E8%A3%85%E7%B3%BB%E5%88%97?lang=zh-CN)  
[ROS setup tutorial | CLion Documentation](https://www.jetbrains.com/help/clion/ros-setup-tutorial.html#example)  
[Windows11 + CLion + WSL2开发ROS - 简书](https://www.jianshu.com/p/685cfab76c6f)  
ROS安装采取鱼香ROS提供的一键安装包，在终端运行  
````shell
wget http://fishros.com/install -O fishros && . fishros
````
随后依照提示更换系统镜像源并安装所需的ROS系统即可，注意，ROS官方最高支持到Ubuntu20.04.6 LTS  
我们安装的版本为ROS-Noetic，它使用的是carkin_make编译系统，下面笔者将演示如何建立一个新的工作区，并在其中开发新的功能包  
让我们先新建个工作区，这里笔者取名为`ros`，当然你也可以在终端操作，使用`mkdir <dirName>` 指令即可，接着我们再在工作区内新建一个`src`文件夹  
![2023-09-17-利用Clion在WSL下进行ROS开发-ROS工作区建立](2023-09-17-利用Clion在WSL下进行ROS开发-ROS工作区建立.png)  
接着我们在工作区目录下运行终端，输入编译指令  
```shell
catkin_make
```
完成工作区的构建后，我们把这个工作空间添加到环境变量  
运行以下指令可以在当前终端将该工作空间添加到环境变量  
```shell
source devel/setup.bash
```
我们可以验证一下看看结果
```shell
echo $ROS_PACKAGE_PATH
```
![2023-09-17-利用Clion在WSL下进行ROS开发-ros包环境变量查看](2023-09-17-利用Clion在WSL下进行ROS开发-ros包环境变量查看.png)
能够看到当前目录，说明我们成功将其添加到了系统环境变量，但是如果新建一个终端则又无法识别了，为了避免每次新建终端都需要重新输入指令添加环境变量，我们将其写入`.bashrc`文件  
将以下指令中的`<your_WorkSpace>`替换为你自己的工作目录的完整路径即可  
```shell
echo "source /<your_WorkSpace>/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
这样就不用每次打开终端都手动输入指令了  
接下来我们进入`src`文件夹创建功能包，这里我们创建一个叫做`my_package`的功能包，其中包括了`roscpp`，`rospy`，`std_msgs`等库  
```shell
cd src
catkin_create_pkg my_package roscpp rospy std_msgs
```
完成功能包创建后，我们直接进入功能包`my_package`下进行工作，在Clion中选择打开，接着选择`my_package`下的`CMakeLists.txt`作为项目打开   
此时会出现项目引导，如果没有出现或者不小心关闭了的话，也可以在设置页，选择构建，执行，部署，然后选择CMake进行修改  
![2023-09-17-利用Clion在WSL下进行ROS开发-CMake修改](2023-09-17-利用Clion在WSL下进行ROS开发-CMake修改.png)  
我们再CMake选项这里填入以下参数，<font color="#c0504d">请自行修改为对应路径</font>  
```text
-DCATKIN_DEVEL_PREFIX=/home/husky/ros/devel 
-DCMAKE_PREFIX_PATH=/home/husky/ros/devel 
-DCMAKE_PREFIX_PATH=/opt/ros/noetic 
-DPYTHON_EXECUTABLE=/usr/bin/python3
```
其中`DCATKIN_DEVEL_PREFIX`对应路径为你工作目录下的devel文件夹   
`DCMAKE_PREFIX_PATH`对应路径参考  
```shell
echo $CMAKE_PREFIX_PATH
```
`PYTHON_EXECUTABLE`的路径一般不用更改  
接下来我们构建目录选择为上一级文件夹下的build文件夹，可以用`../`来表示上一级目录  
![2023-09-17-利用Clion在WSL下进行ROS开发-CMake修改完成](2023-09-17-利用Clion在WSL下进行ROS开发-CMake修改完成.png)  
### cpp文件编译
保存后我们可以在`my_package/src`下写自己的代码了，下面以一个基础的发布-订阅模式来举例，首先我们新建cpp文件`talker`，代码内容如下  
```cpp
//  
// Created by Husky on 2023/9/17.  
//  
#include <sstream>  
#include "ros/ros.h"  
#include "std_msgs/String.h"  
int main(int argc, char **argv)  
{  
    // ROS节点初始化  
    ros::init(argc, argv, "talker");  
    // 创建节点句柄  
    ros::NodeHandle n;  
    // 创建一个Publisher，发布名为chatter的topic，消息类型为std_msgs::String  
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);  
    // 设置循环的频率  
    ros::Rate loop_rate(10);  
    int count = 0;  
    while (ros::ok())  
    {  
        // 初始化std_msgs::String类型的消息  
        std_msgs::String msg;  
        std::stringstream ss;  
        ss << "hello world " << count;  
        msg.data = ss.str();  
        // 发布消息  
        ROS_INFO("%s", msg.data.c_str());  
        chatter_pub.publish(msg);  
        // 循环等待回调函数  
        ros::spinOnce();  
        // 按照循环频率延时  
        loop_rate.sleep();  
        ++count;  
    }  
    return 0;  
}
```
完成代码编写后，在`CMakelists.txt`中加入以下编译规则
```cmake
add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
```
`add_executable(talker src/talker.cpp)`的含义是添加一个名为`talker`的可执行文件，该文件由`src/talker.cpp`源文件编译而成  
`target_link_libraries(talker ${catkin_LIBRARIES})`意味着将`catkin_LIBRARIES`变量中的所有库链接到名为`talker`的目标上。`catkin_LIBRARIES`变量通常包含一系列库文件的路径，这些库文件是在catkin构建系统中定义的。所以，这意味着你的`talker`可执行文件或者库将会链接到这些库中   
点击右上角的锤子尝试构建`talker`  
![2023-09-17-利用Clion在WSL下进行ROS开发-尝试编译](2023-09-17-利用Clion在WSL下进行ROS开发-尝试编译.png)  
可以看到成功的构建并链接到了库中  
![2023-09-17-利用Clion在WSL下进行ROS开发-成功编译](2023-09-17-利用Clion在WSL下进行ROS开发-成功编译.png)  
接下来我们验证下能否正常运行在ROS系统中，首先需要打开ROS  
```shell
roscore
```
接着新建终端运行我们写的`my_package`功能包下的`talker`  
```shell
rosrun my_package talker
```
可以看到我们的代码正常工作了   
![2023-09-17-利用Clion在WSL下进行ROS开发-talker正常工作](2023-09-17-利用Clion在WSL下进行ROS开发-talker正常工作.png)    
订阅者`listener`同理，代码见下，编译方法就留给读者自己实验了    
```cpp
//  
// Created by Husky on 2023/9/17.  
//  
#include "ros/ros.h"  
#include "std_msgs/String.h"  
// 接收到订阅的消息后，会进入消息回调函数  
void chatterCallback(const std_msgs::String::ConstPtr& msg)  
{  
    // 将接收到的消息打印出来  
    ROS_INFO("I heard: [%s]", msg->data.c_str());  
}  
int main(int argc, char **argv)  
{  
    // 初始化ROS节点  
    ros::init(argc, argv, "listener");  
    // 创建节点句柄  
    ros::NodeHandle n;  
    // 创建一个Subscriber，订阅名为chatter的话题，注册回调函数chatterCallback  
    ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);  
    // 循环等待回调函数  
    ros::spin();  
    return 0;  
}
```
### msg文件编译
TODO