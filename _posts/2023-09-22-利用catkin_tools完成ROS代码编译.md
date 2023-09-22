---
title: 【工具使用】利用catkin_tools完成ROS功能包建立
date: 2023-09-22 00:00:00 +0800
img_path: /assets/img/md
categories:
  - 记录
  - 工具使用
tags:
  - 记录
  - 工具使用
---
现在介绍一种使用catkin tools来轻松构建ROS代码构建的方法   
环境为ubuntu20.04.6 LTS  
不涉及到任何IDE的使用，你可以选择任意你喜欢的文本编辑器   
即使是记事本  
官网如下  
[catkin_tools](https://catkin-tools.readthedocs.io/en/latest/installing.html)  
## catkin tools 安装  
鱼香ROS提供的安装环境已包括catkin tools的整合，此处不多赘述  
可通过以下指令安装鱼香ROS  
```bash
wget http://fishros.com/install -O fishros && . fishros
```
## 工作空间创建
首先创建工作空间与对应资源文件夹并进入    
```shell
mkdir <your_WorkSpace>/src
cd <your_WorkSpace>
```
请将尖括号对应内容切换成你自己的目录名称，例如  
```shell
mkdir ros_learn/src
cd ros_learn
```
接着我们执行初始化指令    
```shell
catkin init
catkin build
```
 完成初始化后，我们将环境变量添加到控制台并刷新  
 **请注意这里的`<your_WorkSpace>`是绝对路径**  
 ```shell
echo "source <your_WorkSpace>/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
例如  
 ```shell
echo "source ~/ros_learn/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
到此我们就完成了一个工作空间的创建  
## 功能包创建
让我们先进去`src`文件夹  
```shell
cd src
```
接着我们创建一个名叫`learn_communication`的功能包，并提前引入`roscpp` `rospy` `std_msgs`等包  
```shell
catkin create pkg learn_communication -c roscpp rospy std_msgs
```

然后我们就可以在`learn_communication/src`下编写我们自己的代码了  
```shell
cd learn_communication/src
```
然后我们新建一个`talk.cpp`文件，并粘贴以下代码到其中   
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
保存后我们再在`learn_communication/CMakelists.txt`文件的末尾添加以下内容    
```cmake
add_executable(talk src/talk.cpp)
target_link_libraries(talk ${catkin_LIBRARIES})
```
`add_executable(talker src/talker.cpp)`的含义是添加一个名为`talker`的可执行文件，该文件由`src/talker.cpp`源文件编译而成  
`target_link_libraries(talker ${catkin_LIBRARIES})`意味着将`catkin_LIBRARIES`变量中的所有库链接到名为`talker`的目标上。`catkin_LIBRARIES`变量通常包含一系列库文件的路径，这些库文件是在catkin构建系统中定义的。所以，这意味着你的`talker`可执行文件或者库将会链接到这些库中   
保存后我们在终端尝试构建功能包`learn_communication`  
```shell
catkin build learn_communication
```
等待构建完成后我们可以尝试下运行我们的功能包  
首先运行ROSCORE  
```shell
roscore
```
然后新建一个终端尝试运行我们的功能包功能  
```shell
rosrun learn_communication talk
```
![2023-09-22-利用catkin_tools完成ROS代码编译-运行节点](2023-09-22-利用catkin_tools完成ROS代码编译-运行节点.png)    
很好，我们成功的完成了一个功能包的编译构建~  
接下来`listen.cpp`的构建就留给读者自行完成尝试  
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





