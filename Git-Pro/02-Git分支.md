# Git分支

### 1. 分支简介

`Git`保存的不是文件的变化或者文件的差异，而是一系列不同时刻的文件快照。

创建两个文件：

```shell
$ git init
$ echo "111" > a.txt
$ echo "222" > b.txt
$ git add *.txt
```

`Git`会将整个数据库存储在`.git`目录下，如果此时去查看`.git/objects`目录，你会发现仓库中多了两个`object`

```shell
$ tree .git/objects
.git/objects
├── 58
│   └── c9bdf9d017fcd178dc8c073cbfcbb7ff240d6c
├── c2
│   └── 00906efd24ec5e783bee7f23b5d7c941b0c12c
├── info
└── pack
```

可以使用`git cat-file`命令查看`object`的类型和存储的具体内容：

```shell
$ git cat-file -t 58c9
blob
$ git cat-file -p 58c9
111
```

`blob`类型的对象存储的是一个文件的内容，不包括文件名等其他信息。然后将这些信息经过`SHA1`哈希算法得到对应的哈希值`58c9bdf9d017fcd178dc8c073cbfcbb7ff240d6c`，作为这个`object`在`Git`仓库中的唯一身份证。

此时我们的`Git`仓库是这样子的：



![](.\images\git-object-blob.jpg)



接着执行提交操作：

```shell
$ git commit -am '[+] init'
$ tree .git/objects
.git/objects
├── 0c
│   └── 96bfc59d0f02317d002ebbf8318f46c7e47ab2
├── 4c
│   └── aaa1a9ae0b274fba9e3675f9ef071616e5b209
...
```

这时`Git`仓库中多了两个`object`，使用`cat-file`命令查看类型和内容：

```shell
$ git cat-file -t 4caaa1
tree
$ git cat-file -p 4caaa1
100644 blob 58c9bdf9d017fcd178dc8c0...     a.txt
100644 blob c200906efd24ec5e783bee7...    b.txt
```

`tree`类型的对象是对当前目录结构的一个快照，它存储了一个目录结构，其中包括每个文件或文件夹的权限、类型、对应的`SHA1`值以及文件名。

此时的`Git`仓库是这样的：



![](.\images\git-object-tree.jpg)



紧接着我们发现第三种类型的对象`commit`。它存储的是一个提交的信息，其中包括对应目录结构的快照`tree`的哈希值，上一个提交的哈希值、提交的作者、提交的信息和提交的具体时间。

```shell
$ git cat-file -t 0c96bf
commit
$ git cat-file -p 0c96bf
tree 4caaa1a9ae0b274fba9e3675f9ef071616e5b209
author lzane 李泽帆  1573302343 +0800
committer lzane 李泽帆  1573302343 +0800
[+] init
```

此时的`Git仓库是这样的：`



![](.\images\git-object-commit.png)



此时查看分支信息：

```shell
$ cat .git/HEAD
ref: refs/heads/master

$ cat .git/refs/heads/master
0c96bfc59d0f02317d002ebbf8318f46c7e47ab2
```

此时会发现`master`分支中存储的信息就是上一次提交的`commit`对象的`SHA1`值。在`Git`仓库中，`HEAD`、分支和普通的`tag`可以简单理解为一个指针，指向对应的`commit`的`SHA1`值。



![](.\images\git-branch.png)



从上图可以看出`HEAD`指向`master`，`master`指向`commit`对象。

其实`Git`中还有一个对象那就是`tag`类型的对象，其对应`Git`中的附注标签。

![](.\images\git中的三个区域.jpg)



这里有三个区域，所存储的信息分别为

- 工作目录 （ working directory ）：操作系统上的文件，所有代码开发编辑都在这上面完成。
- 索引（ index or staging area ）：可以理解为一个暂存区域，这里面的代码会在下一次commit被提交到Git仓库。
- Git仓库（ git repository ）：由Git object记录着每一次提交的快照，以及链式结构记录的提交变更历史。