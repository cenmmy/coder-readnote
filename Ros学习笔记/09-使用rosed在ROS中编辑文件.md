# 使用`rosed`在ROS中编辑文件

**描述**: 这个教程展示了如何使用`rosed`使得编辑更加简单。

### 1. 使用rosed

`rosed`是`rosbash`单元的一部分。它允许你直接在包中去直接编辑文件，而不需要提供包的完整的路径。

用法：

```shell
$ rosed [package_name] [filename]
```

例子：

```shell
$ rosed roscpp Logger.msg
```

这个例子演示了如何子roscpp包中编辑Logger.msg文件。

如果这个命令运行失败，原因可能是您没有安装vim编辑器。如果提供的编辑的文件的名称在包中不是唯一的，那么将会提供一个选择列表，你可以选择想要编辑的文件。

### 2. 使用tab补全

通过连续按两次tab键会触发tab补全

### 3. 编辑器

默认的编辑器是vim，对新手更加友好的编辑器是`nano`，在`Ubuntu`中被默认安装，你可以修改.bashrc文件去修改ros默认的编辑器。

```shell
export EDITOR='nano -w'
```