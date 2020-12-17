# 创建一个ROS包

**描述**: 本节教程主要包含了如何使用`catkin`去创建一个包，并且使用`rospack`命令去列出包的依赖。

### 1. catkin包的组成都有什么？

只有满足下面条件的包才能被成为catkin包：

+ 该包必须包含一个与catkin兼容的`package.xml`文件。
+ 该包必须包含使用catkin的`CMakeLists.txt`文件。
+ 每个包必须有其自己的文件夹

一个简单的包的结构可能如下所示：

```shell
my_package/
  CMakeLists.txt
  package.xml
```

### 2. `catkin`工作区空间的包

推荐使用`catkin`工作区空间来进行与`catkin`包相关的工作。但你也可以建立一个独立的`catkin`包。

一般的工作区目录如下所示：

```txt
workspace_folder/        -- WORKSPACE
  src/                   -- SOURCE SPACE
    CMakeLists.txt       -- 'Toplevel' CMake file, provided by catkin
    package_1/
      CMakeLists.txt     -- CMakeLists.txt file for package_1
      package.xml        -- Package manifest for package_1
    ...
    package_n/
      CMakeLists.txt     -- CMakeLists.txt file for package_n
      package.xml        -- Package manifest for package_n
```

### 3. 创建`catkin`工作区

创建`catkin`工作区空间的步骤非常简单

```shell
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws
$ catkin_make
```

`catkin_make`是一个方便操作`catkin`工作区空间的工具。在工作区第一次执行这个命令的时候，它在`src`目录西创建一个`CMakeLists.txt`文件。

此外，`catkin_make`命令还会在当前工作区中创建`build`和`devel`文件夹，在`devel`文件中，你会看到一些脚本文件，执行这些脚本文件，将会覆盖全局的环境变量。

```shell
source devel/setup.bash
```

执行上面的这条命令，确保您的工作区间被设置脚本正确覆盖。这条命令同样会修改`ROS_PACKAGE_PATH`变量，使其包含当前工作空间的目录。

```shell
$ echo $ROS_PACKAGE_PATH
/home/cenmmy/catkin_ws/src:/opt/ros/kinetic/share
```

至此，`catkin`工作区空间就创建完成了。

### 4. 创建一个`catkin`包

这一小节我们将演示如何使用`catkin_create_pkg`脚本去创建一个`catkin`包，并且在创建完成之后你可以做哪些操作。

首先，进入src目录

```shell
cd ~/catkin_ws/src
```

你可以使用`catkin_create_pkg`脚本去创建一个叫做`beginner_tutorial`的包，并且指定其依赖为`std_msgs`, `roscpp`, `rospy`。

```shell
$ catkin_create_pkg beginner_tutorial std_msgs roscpp rospy
```

这条命令将会创建一个名为`beginner_tutorial`的文件夹，这个文件夹中包含`package.xml`和`CMakeLists.txt`文件。

### 5. 编译构建catkin工作区空间并加载设置脚本


```shell
$ cd ~/catkin_ws
$ catkin_make
```

正如在创建`catkin`工作目录时提到的，需要通过加载设置文件，将当前工作区加载到ROS的环境变量中

```shell
source ~/catkin_ws/devel/setup.bash
```

### 6. 包依赖

#### 6.1 一阶依赖

我们在创建`beginner_tutorial`包时指定的依赖就是我们当前包的一阶依赖，可以通过`rospack`命令查看当前包的一阶依赖。

```shell
$ rospack depend1 beginner_tutorial
roscpp
rospy
std_msgs
```

一阶依赖在`package.xml`文件中都有定义。

#### 6.2 间接依赖

所谓的间接依赖无非就是一阶依赖所直接依赖和间接依赖的包，这是一个递归的关系。

通过下面的命令来查看当前包所有的依赖

```shell
$ rospack depends beginner_tutorial
cpp_common
rostime
roscpp_traits
roscpp_serialization
catkin
genmsg
genpy
message_runtime
gencpp
geneus
gennodejs
genlisp
message_generation
rosbuild
rosconsole
std_msgs
rosgraph_msgs
xmlrpcpp
roscpp
rosgraph
ros_environment
rospack
roslib
rospy
```

### 7. 自定义包

#### 7.1 自定义`package.xml`

+ 自定义描述标签
+ 自定义维护者标签
+ 自定义协议标签

上述的这些内容都是用户可以自定义的，用户可以根据自己的实际的情况进行填写。