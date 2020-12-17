# 理解ROS服务和参数

**描述**: 本章教程介绍了ROS服务和参数以及`rosservice`和`rosparam`命令行工具的使用。

### 1. ROS服务

服务是另外一种ROS节点之间进行通信的方式。服务允许节点之间发送请求和接收响应。

### 2. 使用`rosservice`

`rosservice`通过服务可以轻松地附加到ROS客户端/服务端框架上。`rosservice`具有非常多的命令行选项：

```txt
rosservice list     打印已经被激活的服务的信息
rosservice call     使用提供的参数调用服务
rosservice type     打印服务的类型
rosservice find     通过服务类型查找服务
rosservice uri      打印服务的uri
```

#### 2.1 rosservice list

```shell
$ rosservice list
```

#### 2.2 rosservice type

```shell
$ rosservice type [service]
```

#### 2.3 rosservice call

```shell
$ rosservice call [service] [args]
```

### 3. 使用`rosparam`

`rosparam`运行您在参数服务器上存储和操作数据。参数服务器能够存储整型、浮点型、布尔型、字典和列表。`rosparam`使用`yaml`标记语言的语法。在简单的例子总，`yaml`看起来非常的自然，1是整型，1.0是浮点型，`true`是布尔型，`[1, 2, 3]`是整型列表，`{a: b, c: d}`是字典。`rosparam`具有很多命令行选项。

```txt
rosparam set [name] [value]             设置参数
rosparam get [name]                     获取参数
rosparam load [filename] [namespace]    从文件加载参数
rosparam dump [filename] [namespace]    将参数导出到文件中
rosparam delete [name]                  删除参数
rosparam list                           列出参数的名字
```