# Git分支

> Note: 在markdown中使用windows下的路径分割符表示图片路径在`github`网站中不能显示图片。

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



![](./images/git-object-blob.jpg)



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



![](./images/git-object-tree.jpg)



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



![](./images/git-object-commit.png)



此时查看分支信息：

```shell
$ cat .git/HEAD
ref: refs/heads/master

$ cat .git/refs/heads/master
0c96bfc59d0f02317d002ebbf8318f46c7e47ab2
```

此时会发现`master`分支中存储的信息就是上一次提交的`commit`对象的`SHA1`值。在`Git`仓库中，`HEAD`、分支和普通的`tag`可以简单理解为一个指针，指向对应的`commit`的`SHA1`值。



![](./images/git-branch.png)



从上图可以看出`HEAD`指向`master`，`master`指向`commit`对象。

其实`Git`中还有一个对象那就是`tag`类型的对象，其对应`Git`中的附注标签。

![](./images/git中的三个区域.jpg)



这里有三个区域，所存储的信息分别为

- 工作目录 （ working directory ）：操作系统上的文件，所有代码开发编辑都在这上面完成。
- 索引（ index or staging area ）：可以理解为一个暂存区域，这里面的代码会在下一次commit被提交到Git仓库。
- Git仓库（ git repository ）：由Git object记录着每一次提交的快照，以及链式结构记录的提交变更历史。

### 2. 分支的新建与合并

下面我们将以显示中常常遇到的情况作为例子来讲解分支的新建和合并：

1. 开发某个网站。
2. 为实现某个新的需求，创建一个分支。
3. 在这个分支上开展工作。
4. 切换到你的线上分支（production branch）。
2. 为这个紧急任务新建一个分支，并在其中修复它。
3. 在测试通过之后，切换回线上分支，然后合并这个修补分支，最后将改动推送到线上分支。
4. 切换回你最初工作的分支上，继续工作。

假设当前的提交历史如下图所示：

![](./images/新建分支演示图1.png)

首先我们为了实现某个新需求，创建了一个分支`iss53`

```shell
$ git checkout -b iss53
```

该命令创建了新的分支并切换到该分支上。该命令可以分解为：

```shell
$ git branch iss53
$ git checkout iss53
```

![](./images/新建分支演示图2.png)

然后我们在该分支做了一些工作并进行了一次提交

```shell
$ nvim index.html
$ git commit -a -m "added a new file: index.html [iss53]"
```

![](./images/新建分支演示图3.png)

紧接着我接到了那个电话，然后我需要先切换到master分支上，然后再该基础上创建一个新的分支`hotfix`，然后再该分支上针对该问题做修复。

```shell
$ git checkout master
$ git checkout -b hotfix
$ nvim index.html
$ git commit -a -m "fixed the broken email address [hotfix]"
```

![](./images/新建分支演示图4.png)

当我们保证​`​BUG`:bug:已经被修复了，那么我们将可以将该分支合并到主分支上。没有任何冲突的合并再`Git`中称为`fast-forward`快进。

```shell
$ git checkout master
$ git merge hotfix
```

![](./images/新建分支演示图5.png)

接下来我们就可以将`hotfix`分支删除了

```shell
$ git branch -d hotfix
```

![](./images/新建分支演示图6.png)

下面我们可以返回`iss53`分支上继续之前的工作

```shell
$ git checkout iss53
$ nvim index.html
$ git commit -a -m "finished the new footer [iss53]"
```

![](./images/新建分支演示图7.png)

现在我们在分支`iss53`上的工作已经完成了，那么接下来我们就可以将该分支合并到主分支上了。注意，这里的分支合并和之前的分支合并不同，这里的master分支所在的提交并不是`iss53`分支所在的提交的直接祖先，而是两个提交有一个共同的祖先，这里我们需要做一个三方的合并。

```shell
$ git checkout master
$ git merge iss53
```



![](./images/新建分支演示图8.png)

然后删除掉`iss53`分支即可

```shell
$ git branch -d iss53
```

当然上面的三方合并是有可能产生冲突的，比如在分支`iss53`上的`index.html`内容如下

```html
<html>
    <body>
        <h1>
            Hello, Iss53!
        </h1>
    </body>
</html>
```

`master`分支上：

```html
<html>
    <body>
        <h2>
            Hello, Master!
        </h2>
    </body>
</html>
```

公共节点：

```html
<html>
    <body>
        <p>
            Hello, World!
        </p>
    </body>
</html>
```

这时两个分支都对公共节点的内容做了修改，并且修改的地方是相同的，因此在合并时到底保留哪一个更改需要程序员自己做出判断。

```shell
$ git merge iss53
Auto-merging index.html
CONFLICT (content): Merge conflict in index.html
Automatic merge failed; fix conflicts and then commit the result.
```

在手动解决冲突之后，需要使用`git add`命令标记冲突已经解决。然后将暂存区提交形成一个合并节点。

我们可以使用`git mergetool`命令按照提示一步一步解决冲突

```shell
git mergetool
```

然后可以使用`git status`命令来查看冲突是否被解决

```shell
$ git status
On branch master
All conflicts fixed but you are still merging.
	(use "git commit" to conclude merge)
Changes to be committed:
	modified: index.html
```

### 3. 分支管理

#### 3.1 查看分支列表`git branch`

```shell
$ git branch
  iss53
* master
  testing
```

注意使用`*`号标注的分支是现在检出的分支，即`HEAD`指针指向的分支。

#### 3.2 查看每个分支最新一次的提交`git branch -v`

```shell
$ git branch -v
  iss53 93b412c fix javascript issue
* master 7a98805 Merge branch 'iss53'
  testing 782fd34 add scott to the author list in the readmes
```

#### 3.3 查看哪些分支已经合并到当前分支`git branch --merged`

```shell
$ git branch --merged
  iss53
* master
```

因为我们之前合并了iss53分支，因此可以看到该分支在列表中，通常输出中没有星号的分支可以使用`git branch -d`删除掉。

#### 3.4 查看哪些分支还没有合并当前分支`git branch --no-merged`

```shell
$ git branch --no-merged
  testing
```

因为我们还没有合并过`testing`分支，因此该分支显示在列表中，这时如果我们尝试使用`git branch -d`删除分支，则会失败。

```shell
$ git branch -d testing
error: The branch 'testing' is not fully merged.
If you are sure you want to delete it, run 'git branch -D testing'.
```

如果你想要强制删除，使用`git branch -D`