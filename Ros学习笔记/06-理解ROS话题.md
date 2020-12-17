# 理解ROS话题

**描述**: 本教程主要介绍了ROS话题和`rostopic`和`ros plot`的使用

### 1. 设置

#### 1.1 roscore

在开始之前，确保在另外一个终端中运行着`roscore`, 正如前面所述，`roscore`命令运行了Master, rosout和参数服务器节点。

#### 1.2 运行turtlesim_node节点

```shell
$ rosrun turtlesim turtlesim_node
```

#### 1.3 运行turtle_teleop_key节点

```shell
$ rosrun turtlesiim turtle_teleop_key
```

该节点可监听键盘输入，并根据键盘输入控制乌龟运动。

### 2. ROS话题

上面的两个节点之间通过ROS TOPIC进行通信，`turtle_teleop_key`向一个话题发布消息，而`turtlesim_node`从同一个话题接收消息，我们可以通过`rqt_graph`命令行工具来查看两个节点之前的关系。(rqt： ros qt)

```shell
$ rosrun rqt_graph rqt_graph
```

#### 2.1 rostopic echo

```shell
$ rostopic echo [topic]
```

`rostopic echo`用于显示在一个话题上发布的消息。 

例：

```shell
$ rostopic echo /turtle1/cm_vel
```

#### 2.2 rostopic list

```shell
$ rostopic list
```

列出当前被订阅或被发布的所有的话题。

### 3. ROS消息

两个节点之间通过在topic之间发送message进行通信，订阅者和发布者必须使用相同的消息类型，这意味着topic的类型由message的类型来决定。

#### 3.1 使用rostopic type 查看topic中message的类型

```shell
$ rostopic type [topic]
```

上面的命令返回消息的类型

通过rosmsg show查看消息类型的详细定义。

```shell
$ rosmsg show [msg_type]
```

