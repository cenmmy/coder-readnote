# 创建ROS消息和服务

**描述**: 本教程涵盖了如何使用`rosmsg`、`rossrv`和`roscp`等命令行工具去创建和构建ROS消息和服务。

### 1. 消息和服务介绍

+ 消息： 消息文件是一个用来简单描述消息字段的文本文件。他们被用来在不同的语言中为消息生成源代码。
+ 服务：服务文件描述一个服务。它由响应和请求两部分组成。

消息文件被存放在包的消息文件夹中，服务文件被存放在服务文件夹中。

消息文件的每一行对应消息的字段类型和字段名，可用的字段类型如下：

+ int8, int16, int32, int64
+ float32, float64
+ string
+ time, duration
+ 其他的消息文件
+ 变长数组和固定长度的数组

`Header`是一个特殊的类型，`Header`中包含常用的时间戳和坐标帧信息，消息文件的第一行通常是`Header`。

下面是一个消息文件的例子

```txt
Header header
string child_frame_id
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist
```

服务文件和消息文件类似，除了它包含两个部分：请求和响应，这两个部分通过`---`分割。

```txt
int64 A
int64 B
---
int64 Sum
```

A和B是请求，Sum是响应。

### 2. 使用消息

#### 2.1 创建一个消息

让我们在之前创建的包中创建一个消息

```shell
$ roscd beginner_tutorial
$ mkdir msg
$ echo "int64" > msg/Num.msg
```

这个例子的msg文件只有一行，当然你可以创建一个具有多行的更加复杂的msg文件。

```txt
string first_name
string last_name
uint8 age
uint32 score
```

不过还有一个步骤需要做，就是我们需要确保msg文件转换成C++, python或其他语言的代码。

打开package.xml文件，取消下面两个的注释

```xml
  <build_depend>message_generation</build_depend>
  <exec_depend>message_runtime</exec_depend>
```

构建时我们需要message_generation，在运行时我们需要message_runtime。

然后编辑`CMakeLists.txt`，使用`find_package`命令添加`message_generation`依赖。

```cmake
find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   message_generation
)
```

确保导出message运行时依赖

```cmake
catkin_package(
  ...
  CATKIN_DEPENDS message_runtime ...
...)
```

取消下面行的注释，并将`Message1.msg`和`Message2.msg`删除，替换成`Num.msg`。

```cmake
add_message_files(
  FILES
  Num.msg
)
```

通过手动添加msg文件，我们确保CMake知道在添加其他的消息文件之后何时去重新配置项目。

现在我们必须确保`generate_messages`函数被调用。

```cmake
generate_messages(
  DEPENDENCIES
  std_msgs
)
```

### 3 使用srv

#### 3.1 创建服务

```shell
$ roscd beginner_tutorials
$ mkdir srv
```

这次我们不自己手动创建服务文件，而是通过`roscp`命令拷贝其他包中的服务文件。

```shell
$ roscp rospy_tutorials AddTwoInts.srv srv/AddTwoInts.srv
```

与msg类似，我们还需要其他的步骤确保服务文件被转换为相应的代码。

前两个步骤和msg的前两个步骤相同，注意message_generation和message_runtime不仅仅适用于msg，其同样适用于srv。

修改下列代码

```cmake
add_service_files(
  FILES
  AddTwoInts.srv
)
```

这样就配置好了。

### 4. msg和src相同的步骤

添加std_msgs依赖

```cmake
generate_messages(
  DEPENDENCIES
  std_msgs
)
```

重新编译项目生成源代码

```shell
$ roscd beginner_tutorials
$ cd ../..
$ catkin_make
$ cd -
```

### 5. 使用rosmsg和rossrv查看消息和服务

```shell
$ rosmsg show Num
[beginner_tutorial/Num]:
int64 num
$ rossrv show AddTwoInts
[beginner_tutorial/AddTwoInts]:
int64 a
int64 b
---
int64 sum
```