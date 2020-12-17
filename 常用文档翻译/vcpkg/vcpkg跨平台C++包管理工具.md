# vcpkg: 跨平台C++包管理工具

vcpkg是一个C++命令行包管理工具。它极大地简化了在Windows、Linux和macOS上获取和安装第三方二进制库。如果你的项目使用第三方二进制库，我们建议你使用vcpkg去安装它们。vcpkg不仅支持开源库而且支持私有的库。vcpkg中windows目录中所有的库都已经在vs2015、vs2017和vs2019上测试了兼容性。在Windows和Linux/macOS共有的目录中，vcpkg支持超过1900个库，C++社区正在不断地向这两个目录添加更多的库。

### 简单而灵活

通过一条命令，你可以下载源文件去编译一个库。vcpkg自己本身就是一个开源项目，可在Github上访问。它支持自定义你自己的私有vcpkg克隆。例如，指定与公共目录中不同的库或不同版本的库。一台计算机上可以有多个vcpkg克隆。可以将每个设置为生成具有你首先的编译开关的自定义库集合。每个克隆都是一个独立的环境，具有自己的vcpkg.exe副本，该副本仅在自己的层次结构上运行。vcpkg不会添加到任何环境变量，并且不依赖Windows注册表或Visual Studio。

### 源文件而非二进制

对于Windows目录中的库，vcpkg下载的是源文件而非二进制文件。它编译这些源文件使用它能找到的最新版本的Visual Studio。在C++中，应用代码和使用的库使用同一个编译器和编译器版本进行编译时非常重要的。通过使用vcpkg，可以消除或至少大大减少二进制不匹配以及它们可能引起的问题的可能性。在使用特定版本的编译器进行标准化的团队中，一个团队成员可以使用vcpkg下载源代码并编译一组二进制文件。然后它们可以使用export命令压缩这些库和头文件给其他的团队成员。

### 安装

从Github上克隆vcpkg仓库： https://github.com/Microsoft/vcpkg。你可以下载到你喜欢的任何文件夹中。

在vcpkg的根目录中，执行下列的命令：

```shell
bootstrap-vcpkg.vat(Windows)
./bootstrap-vcpkg.sh(Linux, macOS)
```

### 查询可用的库列表

在命令行中键入`vcpkg search`查询有哪些包时可用的。

这个命令枚举vcpkg/ports子文件下的控制文件。

![image-20201123162253354](C:\Users\Lenovo\AppData\Roaming\Typora\typora-user-images\image-20201123162253354.png)

![image-20201123162034590](C:\Users\Lenovo\AppData\Roaming\Typora\typora-user-images\image-20201123162034590.png)

可以通过指定一个模式过滤搜索结果，例如：`vcpkg search opencv`

![image-20201123162446204](C:\Users\Lenovo\AppData\Roaming\Typora\typora-user-images\image-20201123162446204.png)

### 在本地机器上安装一个库

在你通过运行`vcpkg search`获取到一个库的名字之后，你可以运行`vcpkg install`命令去下载并编译一个库。vcpkg使用在ports文件中的库的端口文件。如果目标平台没有指定，vcpkg会在特定的平台编译默认的版本，windwos上编译x86-windows， Linux编译x64-linux, macOS上编译x64-osx。

对于Linux二进制库，vcpkg依赖安装在本地机器上的gcc编译器，在macOS上，vcpkg使用Clang。

在portfile中指定依赖项时， vcpkg也会下载并且安装它们。下载之后，vcpkg构建这个库使用与当前编译库相同的构建系统。首选CMake和MSBuild构建的项目，但Make与其他的任何构建系统一起受支持。如果vcpkg不能在本地机器上找到特定的构建系统，它将会下载并且安装它。

```shell
vcpkg install boost:x86-windows
```

对于CMake项目来说，使用CMAKE_TOOLCHAIN_FILE使得库可以通过`find_package()`命令找到。

例如，在Linux或macOS下：

```shell
cmake .. -DCMAKE_TOOLCHAIN_FILE=vcpkg/scripts/buildsystems/vcpkg.cmake
```

在Windows上：

```shell
cmake .. -DCMAKE_TOOLCHAIN_FILE=vcpkg\scripts\buildsystems\vcpkg.cmake
```

一些库包含可选的安装，例如，当搜索curl库的时候，你将看到方括号中支持的选项列表：

```shell
> vcpkg search curl
cfitsio[curl]                         UseCurl
cpr                  1.5.2            C++ Requests is a simple wrapper around libcurl inspired by the excellent Pyth...
curl                 7.73.0#3         A library for transferring data with URLs
curl[brotli]                          brotli support (brotli)
curl[c-ares]                          c-ares support
curl[http2]                           HTTP2 support
curl[mbedtls]                         SSL support (mbedTLS)
curl[non-http]                        Enables protocols beyond HTTP/HTTPS/HTTP2
curl[openssl]                         SSL support (OpenSSL)
curl[schannel]                        SSL support (Secure Channel)
curl[sectransp]                       SSL support (sectransp)
curl[ssh]                             SSH support via libssh2
curl[ssl]                             Default SSL backend
curl[sspi]                            SSPI support
curl[tool]                            Builds curl executable
curl[winssl]                          Legacy name for schannel
curlpp               2018-06-15-3     C++ wrapper around libcURL
czmq[curl]                            Build with libcurl
fmi4cpp[curl]                         Allows loading FMUs from URL
oatpp-curl           1.2.0#2          Oat++ Modern web framework curl module to use libcurl as a RequestExecutor on ...
restclient-cpp       0.5.2            Simple REST client for C++. It wraps libcurl for HTTP requests.
```

你可以指定在命令行上安装的特定选项。例如， 要使用Windows的默认SSL后端为curl安装库，请使用`vcpkg install curl[ssl]:x86-windows`命令。如果需要，该命令将会安装所有必须的先决条件，包括核心库。

### 与Visual Studio集成

运行`vcpkg integrate install`配置Visual Studio去找出所有的vcpkg头文件和库的路径。如果你有多个vcpkg的克隆，则从此命令运行的克隆将成为新的默认位置。

这时就可以直接在项目中使用库和头文件，而无需其他的配置。

### 导出编译过的二进制和头文件

项目中的每个人去下载并且编译相同的库是低效的。单个团队成员可以使用`vcpkg export`命令创建一个公共的库和头文件的zip文件或一个NuGet包。之后，可以方便得将它分享给团队的其他成员。

### 更新和升级已经安装过的库

公共目录与库的最新版本保持同步。使用`vcpkg update`去判断那个本地库过期。运行`vcpkg upgrade`命令更新端口集合到公共目录中的最新版本。这条命令会自动下载并且重新构建你已经安装过并且过期的库。

默认情况下，`vcpkg upgrade`只会列出所有过期的库。并不会更新它们，添加`--no-dry-run`选项去实际地更新它们。

```shell
> vcpkg upgrade --no-dry-run
```

### 移除库

`vcpkg remove`移除一个安装过的库。如果有其他的库依赖它，你会被询问是否重新运行这个命令使用选项`--recurse`，这将会导致下游的库被移除。

### 更新vcpkg

在vcpkg的root目录下，运行git pull拉取最新的版本。运行`bootstrap-vcpkg`。

### 卸载vcpkg

删除掉对应的文件夹即可。

