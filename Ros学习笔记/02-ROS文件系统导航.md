# ROS文件系统导航

**描述**: 这个教程主要介绍ROS文件系统的概念，包括`roscd`, `rosls`, `rospack`等命令行工具的使用。

`catkin`是ROS的新的构建系统，`rosbuild`是老版本的构建系统，如果你是刚接触`ROS`，推荐你使用`catkin`作为ROS的构建系统。

### 1. 事先准备

安装一个该教程使用的包

```shell
$ sudo apt-get install ros-kinetic-ros-tutorials
```

### 2. 文件系统概念预览

+ Package: package是ROS代码的软件组织单元，每个软件包包含库，可执行文件，脚本和其他的资源。
+ Manifests(package.xml): 清单文件是关于一个包的描述，它用于定义包之间的依赖关系，并捕获包的元信息，如版本，维护人员和许可证等信息。

### 3. 文件系统工具

#### 3.1 rospack

可以通过rospack命令获取包的信息。

```shell
$ rospack find [package-name]
```

例如：

```shell
$ rospack find roscpp
/opt/ros/kinetic/share/roscpp
```

#### 3.2 roscd

可以通过roscd命令直接`cd(change directory)`到指定的包目录下。

用法：

```shell
$ roscd [location-name[/subdir]]
```

例：

```shell
$ roscd roscpp
$ pwd
/opt/ros/kinetic/share/roscpp
```

#### 3.3 rosls

可以通过rosls直接显示包目录下的文件和文件夹列表

用法：

```shell
$ rosls [package-name]
```

例：

```shell
$ rosls roscpp
cmake  msg  package.xml  rosbuild  srv
```

#### 3.4 tab补全