# Ubuntu下安装ROS

不同的ROS版本对应不同的Ubuntu版本，对于Ubuntu16.04而言，我们目前只能安装KINETIC版本。

### 第一步，配置Ubuntu仓库

打开`软件和更新`，然后勾选`restricted`, `universe`, `multiverse`。

### 第二步，设置`source.list`

```shell
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```


### 第三步， 设置key

```shell
$ sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
Executing: /tmp/tmp.rF7IEku0An/gpg.1.sh --keyserver
hkp://keyserver.ubuntu.com:80
--recv-key
C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
gpg: 下载密钥‘AB17C654’，从 hkp 服务器 keyserver.ubuntu.com
gpg: 密钥 AB17C654：公钥“Open Robotics <info@osrfoundation.org>”已导入
gpg: 合计被处理的数量：1
gpg:               已导入：1  (RSA: 1)
```

### 第四步， 安装

```shell
$ sudo apt update
$ sudo apt-get install ros-kinetic-desktop-full
```

### 第五步， 环境设置

```shell
$ echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```

### 第六步， 安装构建包依赖

```shell
$ sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
```

### 第七步， 初始化并更新rosdep

```shell
$ sudo proxychains rosdep init
$ proxychains rosdep update
```

这两条命令在国内必须使用代理进行访问。