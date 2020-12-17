# ELF文件格式描述

ELF（Executable Linkable Format）是Linux系统中采用的目标文件的文件格式，它在COFF（Common File Format）文件格式的基础上经过修改形成。

ELF文件可以分为下面四类：

| ELF文件类型  | 说明                                                         | 实例                                  |
| ------------ | ------------------------------------------------------------ | ------------------------------------- |
| 可重定位文件 | 包含了代码和数据，用来被链接称为可执行文件，静态链接库也可以被分为这一类 | Linux下的.o,Windows下的.obj           |
| 可执行文件   | 可以直接执行的程序                                           | Linux下/bin/bash, Windows下的.exe文件 |
| 共享目标文件 | 与其他的可重定位文件和共享目标文件链接产生新的目标文件       | Linux下的.so文件和WIndows下的.dll文件 |
| 核心转储文件 | Linux进程意外终止时产生的文件                                | Linux下的core dump文件                |

在linux下可以通过`file`命令来查看目标文件属于哪个类型。

### 1. ELF整体结构

![](.\images\elf文件的整体结构.png)

上图是elf文件的整体结构。由elf文件头，各个段，字符串表，符号表和段表组成组成。

### 2. 样例程序

```c++
int printf(const char* format, ... );
  
int global_init_var = 84;
int global_uninit_var;

void func1(int i) {
    printf("%d\n", i);
}

int main(void) {
    static int static_var = 85;
    static int static_var2;

    int a = 1;
    int b;

    func1(static_var + static_var2 + a + b);

    return a;
}
```

通过下列的命令将其编译成目标文件

```shell
gcc -c SimpleSection.c
```

### 3. ELF 文件头

使用`readelf -h SimpleSection.o`命令来查看elf文件的文件头。

```shell
$ readelf -h SimpleSection.o
ELF Header:
  Magic:   7f 45 4c 46 02 01 01 00 00 00 00 00 00 00 00 00 
  Class:                             ELF64
  Data:                              2's complement, little endian
  Version:                           1 (current)
  OS/ABI:                            UNIX - System V
  ABI Version:                       0
  Type:                              REL (Relocatable file)
  Machine:                           Advanced Micro Devices X86-64
  Version:                           0x1
  Entry point address:               0x0
  Start of program headers:          0 (bytes into file)
  Start of section headers:          1184 (bytes into file)
  Flags:                             0x0
  Size of this header:               64 (bytes)
  Size of program headers:           0 (bytes)
  Number of program headers:         0
  Size of section headers:           64 (bytes)
  Number of section headers:         14
  Section header string table index: 13
```

在`/usr/include/elf.h`头文件中定义了结构来表示该文件头

```c
typedef struct
{
  unsigned char e_ident[EI_NIDENT]; /* Magic number and other info EI_NIDENT=16 */
  Elf64_Half    e_type;         /* Object file type */
  Elf64_Half    e_machine;      /* Architecture */
  Elf64_Word    e_version;      /* Object file version */
  Elf64_Addr    e_entry;        /* Entry point virtual address */
  Elf64_Off e_phoff;        /* Program header table file offset */
  Elf64_Off e_shoff;        /* Section header table file offset */
  Elf64_Word    e_flags;        /* Processor-specific flags */
  Elf64_Half    e_ehsize;       /* ELF header size in bytes */
  Elf64_Half    e_phentsize;        /* Program header table entry size */
  Elf64_Half    e_phnum;        /* Program header table entry count */
  Elf64_Half    e_shentsize;        /* Section header table entry size */
  Elf64_Half    e_shnum;        /* Section header table entry count */
  Elf64_Half    e_shstrndx;     /* Section header string table index */
} Elf64_Ehdr;
```

`Ehdr`表示`Elf Header descriptor`, 即ELF 头描述符，Elf64_Ehdr中各个结构体的类型含义如下：

| 类型        | 字长 |
| ----------- | ---- |
| Elf64_Addr  | 8    |
| Elf64_Off   | 8    |
| Elf64_Half  | 4    |
| Elf64_Word  | 4    |
| Elf64_Sword | 4    |

+ `e_ident`是长度为16的无符号字符数组，它的前四个字符表示ELF文件的魔数；第五个字符表示程序的位数，0x01表示32位， 0x02表示64位；第六个字符表示字节序，规定该ELF文件时大端还是小端；第七个字节表示ELF文件的主版本号，一般是1；剩下的9个字符是保留字节。通过分析e_ident，我们可以获取文件头中的这些字段。

  ```txt
    Magic:   7f 45 4c 46 02 01 01 00 00 00 00 00 00 00 00 00 
    Class:                             ELF64
    Data:                              2's complement, little endian
    Version:                           1 (current)
    OS/ABI:                            UNIX - System V
    ABI Version:                       0
  ```

+ `e_type`表示文件类型，`ET_REL`表示可重定位文件；`ET_EXEC`表示可执行文件；`ET_DYN`表示共享目标文件。

+ `e_machine`表示机器类型，`Advanced Micro Devices X86-64`表示该机器上的`cpu`是`AMD`公司生成的`x86-64`架构的。

+ `e_entry`表示elf程序入口的虚拟地址，可重定位文件没有入口地址，一般为0x00。

+ `e_shoff`表示段表在elf文件中的偏移，1184表示段表从第1184个字节开始。

+ `e_ehsize`表示elf文件头的大小。

+ `e_shentsize`段表描述符额的大小。

+ `e_shnum`段表描述符的个数。

+ `e_shstrndx`段表字符串表所在的段在段表中的下标。

### 4. 段表

通过elf文件头中的`e_shoff`字段我们可以获取段表在elf文件中的偏移量。通过`readelf -S SimpleSection.o`命令可以查看段表。

```shell
$ readelf -S SimpleSection.o
There are 14 section headers, starting at offset 0x4a0:

Section Headers:
  [Nr] Name              Type             Address           Offset
       Size              EntSize          Flags  Link  Info  Align
  [ 0]                   NULL             0000000000000000  00000000
       0000000000000000  0000000000000000           0     0     0
  [ 1] .text             PROGBITS         0000000000000000  00000040
       000000000000005f  0000000000000000  AX       0     0     1
  [ 2] .rela.text        RELA             0000000000000000  00000380
       0000000000000078  0000000000000018   I      11     1     8
  [ 3] .data             PROGBITS         0000000000000000  000000a0
       0000000000000008  0000000000000000  WA       0     0     4
  [ 4] .bss              NOBITS           0000000000000000  000000a8
       0000000000000004  0000000000000000  WA       0     0     4
  [ 5] .rodata           PROGBITS         0000000000000000  000000a8
       0000000000000004  0000000000000000   A       0     0     1
  [ 6] .comment          PROGBITS         0000000000000000  000000ac
       000000000000002b  0000000000000001  MS       0     0     1
  [ 7] .note.GNU-stack   PROGBITS         0000000000000000  000000d7
       0000000000000000  0000000000000000           0     0     1
  [ 8] .note.gnu.propert NOTE             0000000000000000  000000d8
       0000000000000020  0000000000000000   A       0     0     8
  [ 9] .eh_frame         PROGBITS         0000000000000000  000000f8
       0000000000000058  0000000000000000   A       0     0     8
  [10] .rela.eh_frame    RELA             0000000000000000  000003f8
       0000000000000030  0000000000000018   I      11     9     8
  [11] .symtab           SYMTAB           0000000000000000  00000150
       00000000000001b0  0000000000000018          12    12     8
  [12] .strtab           STRTAB           0000000000000000  00000300
       000000000000007c  0000000000000000           0     0     1
  [13] .shstrtab         STRTAB           0000000000000000  00000428
       0000000000000074  0000000000000000           0     0     1
Key to Flags:
  W (write), A (alloc), X (execute), M (merge), S (strings), I (info),
  L (link order), O (extra OS processing required), G (group), T (TLS),
  C (compressed), x (unknown), o (OS specific), E (exclude),
  l (large), p (processor specific)
```

`There are 14 section headers, starting at offset 0x4a0`， `0x4a0`换算成10进制之后正好是文件头中的1184。

段表中的第一个段描述符是空的，无意义的。

`/usr/include/elf.h`中同样定义了结构来表示这个结构。

```c++
typedef struct
{
  Elf64_Word    sh_name;        /* Section name (string tbl index) */
  Elf64_Word    sh_type;        /* Section type */
  Elf64_Xword   sh_flags;       /* Section flags */
  Elf64_Addr    sh_addr;        /* Section virtual addr at execution */
  Elf64_Off     sh_offset;      /* Section file offset */
  Elf64_Xword   sh_size;        /* Section size in bytes */
  Elf64_Word    sh_link;        /* Link to another section */
  Elf64_Word    sh_info;        /* Additional section information */
  Elf64_Xword   sh_addralign;       /* Section alignment */
  Elf64_Xword   sh_entsize;     /* Entry size if section holds table */
} Elf64_Shdr;
```

`Shdr`表示`Section Header Descriptor`段头描述符。

+ `sh_name`表示段的名字，实际上是在字符串表中的下表。
+ `sh_type`表示段的类型，`NULL`表示无效段；`PROGBITS`表示数据段、代码段；`SYSTAB`表示符号表段；`STRTAB`表示字符串表段；`RELA`表示重定位段；`NOBITS`表示该段在文件中没有内容，如BSS段；`NOTE`表示该段中保存的是注释性信息。
+ `sh_flags`表示段的标志位，`WRITE`表示该段在进程空间中可写；`ALLOC`表示该段需要在进程空间中分配空间；`EXECINSTR`表示该段在进程空间中可以被执行。

### 5. 各个段

通过段表我们可以找到elf文件中的各个段，接下来让我们看一看各个段存储的内容。

#### （1）代码段

通过`objdump -s -d SimpleSection.o`可以提取出代码段的内容，并且进行反汇编。

```shell
$ objdump -s -d SimpleSection.o
SimpleSection.o:     file format elf64-x86-64

Contents of section .text:
 0000 f30f1efa 554889e5 4883ec10 897dfc8b  ....UH..H....}..
 0010 45fc89c6 488d3d00 000000b8 00000000  E...H.=.........
 0020 e8000000 0090c9c3 f30f1efa 554889e5  ............UH..
 0030 4883ec10 c745f801 0000008b 15000000  H....E..........
 0040 008b0500 00000001 c28b45f8 01c28b45  ..........E....E
 0050 fc01d089 c7e80000 00008b45 f8c9c3    ...........E... 
Contents of section .data:
 0000 54000000 55000000                    T...U...        
Contents of section .rodata:
 0000 25640a00                             %d..            
Contents of section .comment:
 0000 00474343 3a202855 62756e74 7520392e  .GCC: (Ubuntu 9.
 0010 332e302d 31377562 756e7475 317e3230  3.0-17ubuntu1~20
 0020 2e303429 20392e33 2e3000             .04) 9.3.0.     
Contents of section .note.gnu.property:
 0000 04000000 10000000 05000000 474e5500  ............GNU.
 0010 020000c0 04000000 03000000 00000000  ................
Contents of section .eh_frame:
 0000 14000000 00000000 017a5200 01781001  .........zR..x..
 0010 1b0c0708 90010000 1c000000 1c000000  ................
 0020 00000000 28000000 00450e10 8602430d  ....(....E....C.
 0030 065f0c07 08000000 1c000000 3c000000  ._..........<...
 0040 00000000 37000000 00450e10 8602430d  ....7....E....C.
 0050 066e0c07 08000000                    .n......        

Disassembly of section .text:

0000000000000000 <func1>:
   0:   f3 0f 1e fa             endbr64 
   4:   55                      push   %rbp
   5:   48 89 e5                mov    %rsp,%rbp
   8:   48 83 ec 10             sub    $0x10,%rsp
   c:   89 7d fc                mov    %edi,-0x4(%rbp)
   f:   8b 45 fc                mov    -0x4(%rbp),%eax
  12:   89 c6                   mov    %eax,%esi
  14:   48 8d 3d 00 00 00 00    lea    0x0(%rip),%rdi        # 1b <func1+0x1b>
  1b:   b8 00 00 00 00          mov    $0x0,%eax
  20:   e8 00 00 00 00          callq  25 <func1+0x25>
  25:   90                      nop
  26:   c9                      leaveq 
  27:   c3                      retq   

0000000000000028 <main>:
  28:   f3 0f 1e fa             endbr64 
  2c:   55                      push   %rbp
  2d:   48 89 e5                mov    %rsp,%rbp
  30:   48 83 ec 10             sub    $0x10,%rsp
  34:   c7 45 f8 01 00 00 00    movl   $0x1,-0x8(%rbp)
  3b:   8b 15 00 00 00 00       mov    0x0(%rip),%edx        # 41 <main+0x19>
  41:   8b 05 00 00 00 00       mov    0x0(%rip),%eax        # 47 <main+0x1f>
  47:   01 c2                   add    %eax,%edx
  49:   8b 45 f8                mov    -0x8(%rbp),%eax
  4c:   01 c2                   add    %eax,%edx
  4e:   8b 45 fc                mov    -0x4(%rbp),%eax
  51:   01 d0                   add    %edx,%eax
  53:   89 c7                   mov    %eax,%edi
  55:   e8 00 00 00 00          callq  5a <main+0x32>
  5a:   8b 45 f8                mov    -0x8(%rbp),%eax
  5d:   c9                      leaveq 
  5e:   c3                      retq
```

可以看出，代码段就是存储我们编写的代码的段，不过是编译优化之后的代码。

#### （2）数据段和只读数据段

数据段存储`已经初始化`了的全局变量和局部静态变量。

只读数据段存储使用`const`修饰的变量和字符串常量。

使用`objdump -x -s -d SimpleSection.o`来查看符号表中的符号所在段的情况：

```shell
$ objdump -x -s -d SimpleSection.o
SYMBOL TABLE:
0000000000000000 l    df *ABS*  0000000000000000 SimpleSection.c
0000000000000000 l    d  .text  0000000000000000 .text
0000000000000000 l    d  .data  0000000000000000 .data
0000000000000000 l    d  .bss   0000000000000000 .bss
0000000000000000 l    d  .rodata        0000000000000000 .rodata
0000000000000004 l     O .data  0000000000000004 static_var.1920
0000000000000000 l     O .bss   0000000000000004 static_var2.1921
0000000000000000 l    d  .note.GNU-stack        0000000000000000 .note.GNU-stack
0000000000000000 l    d  .note.gnu.property     0000000000000000 .note.gnu.property
0000000000000000 l    d  .eh_frame      0000000000000000 .eh_frame
0000000000000000 l    d  .comment       0000000000000000 .comment
0000000000000000 g     O .data  0000000000000004 global_init_var
0000000000000004       O *COM*  0000000000000004 global_uninit_var
0000000000000000 g     F .text  0000000000000028 func1
0000000000000000         *UND*  0000000000000000 _GLOBAL_OFFSET_TABLE_
0000000000000000         *UND*  0000000000000000 printf
0000000000000028 g     F .text  0000000000000037 main
```

可以看到初始化的静态变量static_var和global_init_var变量被存储在.data段中。

#### （3）BSS段

.bss段存储的是未初始化的全局变量和局部静态变量，但是某些编译器会将未初始化的全局非静态变量定义成`COMMON`符号，而未初始化的全局静态变量一定是存储在.bss段中的， 除此之外，初始化为0的静态变量同样会被存储在.bss段中，这是编译器进行的一种优化。

#### （4）重定位表

我们注意到段表中有一个名为`.rel.text`的段，它的类型为`REL`，表示它是一个重定位表，重定位表中存储的是需要重定位的一些信息，每个需要重定位的代码段和数据段都有一个相应的重定位表。重定位表的头描述符中的info字段表示其作用的段，如`.rel.text`的`info`字段为1,而段表中下表为1的段正是`text`段。

#### (5) 字符串表

字符串表用来表示段名和变量名等，段名的存储方式如下图所示：

![](.\images\字符串表.png)

当其他段中引用字符串段表中的字符串的时候，只需要提供字符串在字符串表中开始的下表即可，而无需提供结尾下表，因为段表中的字符串都是以\0结尾的。

常见的字符串表的为`.strtab`和`.shstrtab`分别表示字符串表和段表字符串表，字符串表存储的普通字符串，而段表字符串表存储的是段表中用到的字符串，通常存储的是段名。

#### （6）符号表

符号表的结构同样在`/usr/include/elf.h`中存在定义，如下：

```c++
typedef struct
{
  Elf64_Word    st_name;        /* Symbol name (string tbl index) */
  unsigned char st_info;        /* Symbol type and binding */
  unsigned char st_other;       /* Symbol visibility */
  Elf64_Section st_shndx;       /* Section index */
  Elf64_Addr    st_value;       /* Symbol value */
  Elf64_Xword   st_size;        /* Symbol size */
} Elf64_Sym;
```

+ `st_name`: 符号名，为字符串表中的下标。
+ `st_info`: 符号的类型和绑定信息，高四位表示符号的绑定信息，`LOCAL`表示局部符号，`GLOBAL`表示全局符号，`WEAK`表示弱符号。低四位表示符号的类型，`NO_TYPE`表示未知符号类型； `OBJECT`表示该符号是一个数据对象，如变量和数组； `FUNC`表示该符号是个函数或其他可执行代码；`SECTION`表示该符号是个段；`FILE`表示该符号表示文件名。
+ `st_shndx`：表示该符号所在段在段表中的下标，对于不是定义在本目标文件中或特殊符号，`ABS`表示该符号包含了一个绝对的值；`COMMON`表示该符号是一个`COMMON块`的符号，一般来说，为初始化的全局非静态符号就是这种类型的；`UNDEF`表示该符号在该目标文件中被引用，但未在该目标文件中进行定义，其定义在其他的目标文件中。
+ `st_value`：表示符号的地址信息。

通过`readelf -s SimpleSection.o`命令来查看符号的相关信息

```shell
$ readelf -s SimpleSection.o
Symbol table '.symtab' contains 18 entries:
   Num:    Value          Size Type    Bind   Vis      Ndx Name
     0: 0000000000000000     0 NOTYPE  LOCAL  DEFAULT  UND 
     1: 0000000000000000     0 FILE    LOCAL  DEFAULT  ABS SimpleSection.c
     2: 0000000000000000     0 SECTION LOCAL  DEFAULT    1 
     3: 0000000000000000     0 SECTION LOCAL  DEFAULT    3 
     4: 0000000000000000     0 SECTION LOCAL  DEFAULT    4 
     5: 0000000000000000     0 SECTION LOCAL  DEFAULT    5 
     6: 0000000000000004     4 OBJECT  LOCAL  DEFAULT    3 static_var.1920
     7: 0000000000000000     4 OBJECT  LOCAL  DEFAULT    4 static_var2.1921
     8: 0000000000000000     0 SECTION LOCAL  DEFAULT    7 
     9: 0000000000000000     0 SECTION LOCAL  DEFAULT    8 
    10: 0000000000000000     0 SECTION LOCAL  DEFAULT    9 
    11: 0000000000000000     0 SECTION LOCAL  DEFAULT    6 
    12: 0000000000000000     4 OBJECT  GLOBAL DEFAULT    3 global_init_var
    13: 0000000000000004     4 OBJECT  GLOBAL DEFAULT  COM global_uninit_var
    14: 0000000000000000    40 FUNC    GLOBAL DEFAULT    1 func1
    15: 0000000000000000     0 NOTYPE  GLOBAL DEFAULT  UND _GLOBAL_OFFSET_TABLE_
    16: 0000000000000000     0 NOTYPE  GLOBAL DEFAULT  UND printf
    17: 0000000000000028    55 FUNC    GLOBAL DEFAULT    1 main
```

