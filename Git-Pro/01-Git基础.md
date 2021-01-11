# Git基础

### 1. 获取Git仓库

获取Git项目仓库的方法有两种：

+ 将现有项目或目录下导入所有文件到Git中
+ 从一个服务器克隆一个现有的仓库

#### 1.1 将现有项目或目录下导入所有文件到Git中

```shell
$ git init
```

该命令在当前文件夹创建一个`.git`子目录，这个子目录中包含了Git仓库初始化所需的文件。

#### 1.2 从一个服务器克隆一个现有的仓库

```shell
$ git clone [url]
```

该命令会将远程仓库克隆到本地。

### 2. 记录每次更新到仓库

![](./images/文件的状态变化周期.png)



工作目录下的文件共有两种状态：已跟踪和未跟踪。已跟踪的文件是指被纳入了版本控制的文件，在上一次快照中有它们的记录。如上图所示，

+ 未跟踪的文件通过`git add <file>`命令转变未暂存的文件；
+ 暂存的文件通过`git commit`命令转变为未修改的文件；
+ 当我们编辑了文件之后，未修改的文件会转变为已修改的文件
+ 已修改的文件通过`git add <file>`将修改的部分提交到暂存区，文件状态转变为暂存状态；
+ 未修改状态下的文件可以通过`git rm <file>`命令从版本控制中删除。

#### 2.1 检查当前文件的状态`git status`

```shell
$ echo 'My Project' > README
$ git status
On branch master
Untracked files:
	(use "git add <file>..." to include in what will be committed)
	README
nothing added to commit but untracked files present (use "git add" to track)
```

#### 2.2 跟踪新文件`git add`

```shell
$ git add README
$ git status
On branch master
Changes to be committed
	(use "git reset HEAD <file>..." to unstage)
	new file: README
```

#### 2.3 暂存已修改的文件`git add`

```shell
$ git status
On branch master
Changes to be committed:
	(use "git reset HEAD <file>..." to unstage)
	new file: README
Changes not staged for commit:
	(use "git add <file>..." to update what will be committed)
	(use "git checkout -- <file>..." to discard changes in working
directory)
modified: CONTRIBUTING.md
$ git add CONTRIBUTING.md
$ git status
On branch master
Changes to be committed:
	(use "git reset HEAD <file>..." to unstage)
	new file: README
	modified: CONTRIBUTING.md
```

`git add`命令是一个多功能命令：

+ 跟踪新文件
+ 将已跟踪的文件放到暂存区
+ 合并时将有冲突的文件标记为已解决状态

#### 2.4 忽略文件`.gitignore`

我们通常忽略将项目运行、编译过程中创建的临时文件加到git中，因此我们可以创建一个名为`.gitignore`的文件，`.gitignore`的格式规范如下：

+ 所有空行或者以#号开头的行都会被Git忽略
+ 可以使用标准的glob模式匹配
+ 匹配模式可以以`/`开头防止递归
+ 匹配模式可以以`/`结尾指定目录
+ `!`表示取反

#### 2.5 查看已暂存和未暂存的修改`git diff`

`git diff`命令的作用有两个：

+ 比较当前目录与暂存区的区别
+ 比较暂存区与本地git仓库的区别

##### 2.5.1 比较当前目录与暂存区的区别`git diff`

```shell
$ git diff
```

##### 2.5.2 比较暂存区与本地git仓库的区别`git diff --cached`

```shell
$ git diff --staged
```

#### 2.6 提交更新`git commit`

```shell
$ git commit
```

这一步的作用是将暂存区中的文件保存到本地git仓库中。

#### 2.7 跳过使用暂存区域`git commit -a`

```shell
$ git commit -a
```

这条命令会先将已跟踪的文件的更改提交到暂存区，然后将暂存区提交到本地仓库。

#### 2.8 移除文件`git rm`

```shell
$ git rm <file>
```

该命令会将指定的文件从跟踪列表中删除，并将其从工作目录中删除，这样该文件以后就不会出现在未跟踪列表中了。

如果只是在工作目录中删除了该文件，那么运行`git status`命令会在出现形如下面的输出

```shell
$ rm PROJECTS.md
$ git status
On branch master
Your branch is up-to-date with 'origin/master'.
Changes not staged for commit:
	(use "git add/rm <file>..." to update what will be committed)
	(use "git checkout -- <file>..." to discard changes in working
directory)
	deleted: PROJECTS.md
no changes added to commit (use "git add" and/or "git commit -a")
```

这时我们需要使用`git rm <file>`命令记录此次的移除文件的操作，因此为了方便，在移除文件的时候，尽量直接使用`git rm <file>`命令直接删除。

如果在删除之前，文件被修改过并且添加到了暂存区，在删除的时候我们需要执行下面的命令

```shell
$ git rm -f <file>
```

这是git为了我们误删还没有添加到快照总的数据的安全措施。

如果我们误将不希望添加到跟踪列表的文件添加到了跟踪列表，可以执行下面的命令

```shell
$ git rm --cached <file>
```

然后在`.gitignore`文件中将该文件加入。

#### 移动文件`git mv`

```shell
$ git mv <file>
```

该命令常常被用来修改文件名称。

### 3. 查看提交历史`git log`

直接运行`git log`命令会将每次提交的`SHA-1`校验和、作者的名字和电子邮件的地址、提交时间和提交说显示出来。

```shell
$ git log
commit ca82a6dff817ec66f44342007202690a93763949
Author: Scott Chacon <schacon@gee-mail.com>
Date: Mon Mar 17 21:52:11 2008 -0700

	changed the version number

commit 085bb3bcb608e1e8451d4b2432f8ecbe6306e7e7
Author: Scott Chacon <schacon@gee-mail.com>
Date: Sat Mar 15 16:40:33 2008 -0700

	removed unnecessary test

commit a11bef06a3f659402fe7563abf99ad00de2209e6
Author: Scott Chacon <schacon@gee-mail.com>
Date: Sat Mar 15 10:31:28 2008 -0700

	first commit
```

#### 3.1 显示每次提交的内容差异`git log -p`

```shell
$ git log -p -2
commit ca82a6dff817ec66f44342007202690a93763949
Author: Scott Chacon <schacon@gee-mail.com>
Date: Mon Mar 17 21:52:11 2008 -0700
changed the version number
diff --git a/Rakefile b/Rakefile
index a874b73..8f94139 100644
--- a/Rakefile
+++ b/Rakefile
@@ -5,7 +5,7 @@ require 'rake/gempackagetask'
spec = Gem::Specification.new do |s|
s.platform = Gem::Platform::RUBY
s.name = "simplegit"
- s.version = "0.1.0"
+ s.version = "0.1.1"
s.author = "Scott Chacon"
s.email = "schacon@gee-mail.com"
s.summary = "A simple gem for using Git in Ruby code."
commit 085bb3bcb608e1e8451d4b2432f8ecbe6306e7e7
Author: Scott Chacon <schacon@gee-mail.com>
Date: Sat Mar 15 16:40:33 2008 -0700
removed unnecessary test
diff --git a/lib/simplegit.rb b/lib/simplegit.rb
index a0a60ae..47c6340 100644
--- a/lib/simplegit.rb
+++ b/lib/simplegit.rb
@@ -18,8 +18,3 @@ class SimpleGit
end
end
-
-if $0 == __FILE__
- git = SimpleGit.new
- puts git.show
-end
\ No newline at end of file
```

由于`git log -p` 的输出内容通常较多，因此我们通常在其后加上`-n`参数表示希望输出的行数，如上命令添加参数`-2`，表示希望输出两行。

#### 3.2 显示每次提交的简略统计信息`git log --stat`

```shell
$ git log --stat
commit ca82a6dff817ec66f44342007202690a93763949
Author: Scott Chacon <schacon@gee-mail.com>
Date: Mon Mar 17 21:52:11 2008 -0700

	changed the version number

Rakefile | 2 +-
1 file changed, 1 insertion(+), 1 deletion(-)

commit 085bb3bcb608e1e8451d4b2432f8ecbe6306e7e7
Author: Scott Chacon <schacon@gee-mail.com>
Date: Sat Mar 15 16:40:33 2008 -0700

	removed unnecessary test

lib/simplegit.rb | 5 -----
1 file changed, 5 deletions(-)

commit a11bef06a3f659402fe7563abf99ad00de2209e6
Author: Scott Chacon <schacon@gee-mail.com>
Date: Sat Mar 15 10:31:28 2008 -0700

	first commit

README | 6 ++++++
Rakefile | 23 +++++++++++++++++++++++
lib/simplegit.rb | 25 +++++++++++++++++++++++++
```

上述命令显示所有被修改的文件，有多少文件被修改了，总共删除了多少行，新增了多少行。

#### 3.3 每次提交显示一行`git log --oneline`

```shell
$ git log --oneline
fd6f8f7 (HEAD -> master, origin/master) fix :bug:
cf3ec57 :cow:
8810109 修改图片路径为相对路径，解决图片不显示的问题
79b4417 新增获取git仓库和记录每次更新到仓库的方法
5cf5dce 新增在CMake中通过CTest进行单元测试
f11c02a hello, I'm :pig:
78789f2 新增检测Python模块和包
6f13f2d 新增检测Python库
ec2e873 新增find_package的用法
64d24ea 新增处理与编译器相关及处理器体系结构相关的代码的方法
b666600 新增CMake处理与平台相关代码的方法
51b0e4d 新增在CMake中检测操作系统的信息
98b62cf 新增CMAKE使用控制流的方法
b92976b MAKE为语言设置标准的方法
1062830 新增CMAKE设置编译选项方法
822a0b8 新增切换构建类型
cc58885 指定编译器版本
a9b9e72 新增向用户显示选项
4b4f205 新增用条件语句控制编译
```

#### 3.4 形象地显示分支和合并历史`git log --graph`

```shell
$ git log --graph -5
* commit fd6f8f7566a5f4c2a04e8964a9b439a43111a818 (HEAD -> master, origin/master)
| Author: cenmmy <2282805045@qq.com>
| Date:   Mon Jan 11 09:08:54 2021 +0800
|
|     fix :bug:
|
* commit cf3ec57e8557cba71480b69d3f87e23e0923a69d
| Author: cenmmy <2282805045@qq.com>
| Date:   Mon Jan 11 09:03:05 2021 +0800
|
|     :cow:
|
* commit 88101093447e204911943dcf3b46de3e6d22230f
| Author: cenmmy <2282805045@qq.com>
| Date:   Mon Jan 11 09:01:00 2021 +0800
|
|     修改图片路径为相对路径，解决图片不显示的问题
|
* commit 79b441773ad1427fe9fb10c2a5d1cf084108ac6f
| Author: cenmmy <2282805045@qq.com>
| Date:   Sun Jan 10 17:58:15 2021 +0800
|
|     新增获取git仓库和记录每次更新到仓库的方法
|
* commit 5cf5dce20654bb32691aaa4f2401ce5e220186f4
| Author: cenmmy <2282805045@qq.com>
| Date:   Sat Jan 9 17:08:46 2021 +0800
|
|     新增在CMake中通过CTest进行单元测试
$ git log --graph --oneline
* 7e6bc04 新增代码膨胀章节
* a6426b3 新增创建发布者内容
*   03d4165 创建合并
|\
| * 4b3de44 新增C++模板中概念
* | f02de83 Merge remote-tracking branch 'origin/master'
|\|
| *   45304f1 Merge remote-tracking branch 'origin/master'
| |\
| * | 1789659 新增C++拾遗/typename的用法
* | | 8f9e1fe 新增创建ROS消息和服务章节
| |/
|/|
* | c2018ef 新增rosed的用法
|/
*   ea54e73 Merge remote-tracking branch 'origin/master'
|\
| * 7fb6fd9 新增roslaunch命令和launch文件的格式
* | 5ec91f1 新增容器迭代器和算法章节
|/
* af6d2bc 新增对动态库依赖静态库的思考
* 93817e2 新增理解ROS服务和参数章节
* b7d9eff 新增ROS学习笔记
* 21515a6 新增模板参数类型详解章节
* b959b00 初始化
```

#### 3.5 限制输出长度`git log --since`,`git log --until`

##### 3.5.1 显示最近两周内的提交`git log --since=2.weeks`

```shell
$ git log --since=2.weeks
commit fd6f8f7566a5f4c2a04e8964a9b439a43111a818 (HEAD -> master, origin/master)
Author: cenmmy <2282805045@qq.com>
Date:   Mon Jan 11 09:08:54 2021 +0800

    fix :bug:

commit cf3ec57e8557cba71480b69d3f87e23e0923a69d
Author: cenmmy <2282805045@qq.com>
Date:   Mon Jan 11 09:03:05 2021 +0800

    :cow:

commit 88101093447e204911943dcf3b46de3e6d22230f
Author: cenmmy <2282805045@qq.com>
Date:   Mon Jan 11 09:01:00 2021 +0800

    修改图片路径为相对路径，解决图片不显示的问题

commit 79b441773ad1427fe9fb10c2a5d1cf084108ac6f
Author: cenmmy <2282805045@qq.com>
Date:   Sun Jan 10 17:58:15 2021 +0800

    新增获取git仓库和记录每次更新到仓库的方法

commit 5cf5dce20654bb32691aaa4f2401ce5e220186f4
Author: cenmmy <2282805045@qq.com>
Date:   Sat Jan 9 17:08:46 2021 +0800

    新增在CMake中通过CTest进行单元测试
```

##### 3.5.2 显示指定时间之前的提交`git log --until`

```shell
$ git log --until=2020-01-01
commit aeb85609514ddf14df70c03cc7b5e29c399385c1
Author: cenmmy <2282805045@qq.com>
Date:   Thu Dec 31 16:35:08 2020 +0800

    新增记录并回放数据的方式

commit a19673f55195d4401247732bcb70748e81c2bf85
Author: cenmmy <2282805045@qq.com>
Date:   Wed Dec 30 22:06:19 2020 +0800

    C++中元函数的实现

commit 1df765b510ac8e3bee5d76e5d06ff4c099e45f1e
Author: cenmmy <2282805045@qq.com>
Date:   Mon Dec 28 19:49:19 2020 +0800

    添加测试服务端和客户端教程

commit 198eaf40296ac8a6b7695e6fe4ad74ebca9e66e1
Author: cenmmy <2282805045@qq.com>
Date:   Sun Dec 27 20:11:53 2020 +0800

    新增模板递归

commit 61584cf690bd075e630fe0eb7f8b44f15ce9abb8
Author: cenmmy <2282805045@qq.com>
Date:   Sat Dec 26 16:55:33 2020 +0800

    新增策略在模板中的使用
```

##### 3.5.3 显示指定作者的提交`git log --author="cenmmy"`

```shell
$ git log --author="cenmmy" -5
commit fd6f8f7566a5f4c2a04e8964a9b439a43111a818 (HEAD -> master, origin/master)
Author: cenmmy <2282805045@qq.com>
Date:   Mon Jan 11 09:08:54 2021 +0800

    fix :bug:

commit cf3ec57e8557cba71480b69d3f87e23e0923a69d
Author: cenmmy <2282805045@qq.com>
Date:   Mon Jan 11 09:03:05 2021 +0800

    :cow:

commit 88101093447e204911943dcf3b46de3e6d22230f
Author: cenmmy <2282805045@qq.com>
Date:   Mon Jan 11 09:01:00 2021 +0800

    修改图片路径为相对路径，解决图片不显示的问题

commit 79b441773ad1427fe9fb10c2a5d1cf084108ac6f
Author: cenmmy <2282805045@qq.com>
Date:   Sun Jan 10 17:58:15 2021 +0800

    新增获取git仓库和记录每次更新到仓库的方法

commit 5cf5dce20654bb32691aaa4f2401ce5e220186f4
Author: cenmmy <2282805045@qq.com>
Date:   Sat Jan 9 17:08:46 2021 +0800

    新增在CMake中通过CTest进行单元测试
```

##### 3.5.4 显示包含指定关键字的提交`git log --grep`

```shell
$ git log --grep="CMake"
commit 5cf5dce20654bb32691aaa4f2401ce5e220186f4
Author: cenmmy <2282805045@qq.com>
Date:   Sat Jan 9 17:08:46 2021 +0800

    新增在CMake中通过CTest进行单元测试

commit b66660012ce7a30f0286c32d22aa6d200cacab17
Author: cenmmy <2282805045@qq.com>
Date:   Mon Jan 4 14:55:56 2021 +0800

    新增CMake处理与平台相关代码的方法

commit 51b0e4d663c93860799fb00fe950e24b98773485
Author: cenmmy <2282805045@qq.com>
Date:   Mon Jan 4 14:34:58 2021 +0800

    新增在CMake中检测操作系统的信息

commit 9bf358b8411f393dcafaf09d8b6d32f35e87f9f9
Author: cenmmy <2282805045@qq.com>
Date:   Fri Jan 1 15:42:07 2021 +0800

    新增CMake菜谱读书笔记

```

