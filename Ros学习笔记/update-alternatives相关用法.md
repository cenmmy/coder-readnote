# update-alternatives的相关用法

该命令用于维护链接符号，假设我们需要不同版本的gcc来编译不同的项目，可以使用这个工具修改相同的gcc链接符号。下面来具体的讲解该命令的相关用法。

### 1. 添加一个链接

```shell
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-5
```

上面的命令创建了一个链接，该链接位于/usr/bin/gcc，这个链接对应的名字为gcc，该链接可选的链接对象有/usr/bin/gcc-5

我们还可以为该链接添加另外一个可选的对象，比如`/usr/bin/gcc-7`

```shell
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7
```

我们可以在gcc-5和gcc-7之间进行切换，当项目以来gcc-5的时候，将其切换成为gcc-5, 其他情况可以切换成为gcc-7

我们还可以设置链接对象的优先级，以修改默认情况的版本。

```shell
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-5 50
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 70
```

我们分别将两个链接对象的优先级设置为50和70, 在不设置的情况下，默认的版本会是gcc-7

通常gcc和g++的版本需要保持一致，我们可以通过下面的方法使得在修改gcc的版本的同时修改g++的版本

```shell
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-5 50 --slave /usr/bin/g++ g++ /usr/bin/g++-5
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 70 --slave /usr/bin/g++ g++ /usr/bin/g++-7
```

接下来我们可以配置gcc和g++的版本了

```shell
$ sudo update-alternatives --config gcc
有 2 个候选项可用于替换 gcc (提供 /usr/bin/gcc)。

  选择       路径          优先级  状态
------------------------------------------------------------
* 0            /usr/bin/gcc-7   70        自动模式
  1            /usr/bin/gcc-5   60        手动模式
  2            /usr/bin/gcc-7   70        手动模式

要维持当前值[*]请按<回车键>，或者键入选择的编号：0

$ gcc --version
gcc (Ubuntu 7.5.0-3ubuntu1~16.04) 7.5.0
Copyright (C) 2017 Free Software Foundation, Inc.
This is free software; see the source for copying conditions.  There is NO
warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
```

### 2. 删除链接路径
```shell
sudo update-alternatives --remove gcc /usr/bin/gcc-5
```

通过这条命令我们就删除了gcc-5选项了，还可以执行--remove-all参数删除全部链接路径