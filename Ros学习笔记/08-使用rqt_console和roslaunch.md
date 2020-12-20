# 使用rqt_console和roslaunch

**描述**: 本章教程讲解了使用`rqt_console`和`rqt_logger_level`进行调试，讲解了通过`roslaunch`命令一次性启动多个节点。

### 1. 使用`rqt_console`和`rqt_logger_level`

`ros_console`附加到ROS的日志框架上被用来输出节点的日志信息，`rqt_logger_level`被用来改变节点的日志输出级别。

```shell
$ rosrun rqt_console rqt_console
$ rosrun rqt_logger_level rqt_logger_level
```

`rqt_console`用来显示节点的日志输出

`rqt_logger_level`用来设置日志的输出级别，可以为不同的节点设置不同的日志级别。

日志级别总共有5个，从上到下，日志级别依次增强

+ Debug
+ Error
+ Warn
+ Info
+ Debug

### 2. 使用roslaunch

`roslaunch`用于启动定义在launch文件中的节点。

用法：

```shell
$ roslaunch [package] [filename.launch]
```

首先cd到先前创建的`beginner_tutorial`节点中

```shell
$ cd ~/catkin_ws
$ source devel/setup.bash
$ roscd beginner_tutorial
```

然后创建launch文件夹

```shell
$ mkdir launch
$ cd launch
```

| 存储launch文件的文件夹的名称不必非得命名为launch，实际上，你甚至不必将launch文件存储在一个单独的文件夹中。roslaunch命令会自动查找指定的包下可用的launch文件，然而在单独的文件夹中存储launch文件是一个好的选择。

#### 2.1 启动文件说明

启动文件的文件格式为xml，最外面的标签为launch

```xml
<launch>
    <group ns="turtlesim1">
        <node pkg="turtlesim" name="sim", type="turtlesim_node" />
    </group>
    <group ns="turtlesim2">
        <node pkg="turtlesim" name="sim", type="turtlesim_node" />
    </group>
    <node pkg="turtlesim" name="mimic" type="mimic">
        <remap from="input" to="turtlesim1/turtle1" />
        <remap from="output" to="turtlesim2/turtle1" />
    </node>
</launch>
```

上图的lanuch文件中，我们首先声明了两个group，这两个group的命名空间分别是turtlesim1和turtlesim2，由于命名空间的存在，我们在两个不同的命名空间中声明两个相同的节点是不会产生冲突的。

第三个节点将topic的输入和输出重命名为了turtlesim1/turtle1和turtlesim2/turtle2。


