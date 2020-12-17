# 理解ROS节点

**描述**: 本章教程主要讲解了ROS图的概念并且讨论了`roscore`, `rosnode`和`rosrun`命令行工具的使用。

### 1. 图概念快速预览

+ **Node**: 一个节点是一个可执行文件，节点之间通过ROS来进行通信。
+ **Message**: 消息，在向订阅和发布一个话题时使用。
+ **Topic**: 话题，节点可以向一个话题发布消息或从一个订阅的话题接受消息。
+ **Master**: ROS的名称服务。
+ **rosout**: ROS中的标准输出和标准错误输出。
+ **roscore**: Master, rosout 和 参数服务器

### 2. roscore

当使用ROS的时候，首先应该运行`roscore`命令。

```shell
$ roscore
... logging to /home/cenmmy/.ros/log/605c42c4-3db7-11eb-a815-1008b1ce0573/roslaunch-cmy-4918.log
Checking log directory for disk usage. This may take awhile.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://cmy:43837/
ros_comm version 1.12.17


SUMMARY
========

PARAMETERS
 * /rosdistro: kinetic
 * /rosversion: 1.12.17

NODES

auto-starting new master
process[master]: started with pid [4929]
ROS_MASTER_URI=http://cmy:11311/

setting /run_id to 605c42c4-3db7-11eb-a815-1008b1ce0573
process[rosout-1]: started with pid [4942]
started core service [/rosout]
```

### 3. 使用rosnode

```shell
$ rosnode list
/rosout
```

```shell
$ rosnode info /rosout
--------------------------------------------------------------------------------
Node [/rosout]
Publications: 
 * /rosout_agg [rosgraph_msgs/Log]

Subscriptions: 
 * /rosout [unknown type]

Services: 
 * /rosout/get_loggers
 * /rosout/set_logger_level


contacting node http://cmy:37715/ ...
Pid: 5104
```

### 4. 使用rosrun

`rosrun`允许您直接使用报名来运行包中的节点。

Usage:

```shell
$ rosrun [package_name] [node_name]
```

我们可以运行`turtlesim`包中的节点来测试运行

```shell
$ rosrun turtlesim turtlesim_node
$ rosnode list
/rosout
/turtlesim
```

你可以对节点进行重命名

```shell
$ rosrun turtlesim turtlesim_node __name:=my_turtle
$ rosnode list
/my_turtle
/rosout
```



