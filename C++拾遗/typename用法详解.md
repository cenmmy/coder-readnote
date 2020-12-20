# `typename`用法详解

### 1. 起因

最近在阅读《深入实践C++模板编程》这本书的时候，在第六章中出现下列的代码示例

```c++
typedef typename __type_traits<T>::has_trivial_destructor trivial_destructor;
```

这段代码看了半天也没看懂，之前`typedef`的用法通常是如下所示

```c++
typedef [old_type] [new_type]
```

上面的代码中使用的`typename`作用是什么呢？

按照C++官方手册中的介绍，`tpyename`的用法之一就是用于声明某个带决策的有限定名是类型。即上面的代码中`vec::value_type`是类型。

### 2. `typename`的来源

`typename`关键字是后来被引入的，一开始在模板参数定义的时候，对于类型模板参数，使用的是`class`关键字进行修饰，但是`class`关键字同样具有表示用于自定义类型的含义，而使用`class`关键字修饰的类型在模板参数中可以是`C++`的内置类型，这里就有些奇怪。这时促使标准委员会引入新关键字的一个因素。

不适用`typename`可能会在声明某个带决策的有限定名的时候产生歧义。请看如下的代码：

```c++
template<class T>
void foo() {
    T::iterator* iter;
}
```

上面这段代码的目的是什么？大多数人的第一反应是在模板函数中声明了一个指向T::iterator类型的指针。传入的模板实参可能的定义如下：

```c++
struct ContainsAType {
    struct iterator {
        ...
    }
    ...
}
```

如果传入的模板实参是这样的话，代码逻辑没有任何的问题，但是如果传入的模板实参为：

```c++
struct ContainsAType {
    static int iterator;
}
```

那么之前的代码就会被编译器当成乘法表达式，当然编译器会报错，因为`iter`未定义。但是如果存在一个全局变量`iter`，那么代码就没有任何的问题。

同一行代码能以两种完全不同的方式解释，而且在模板实例化之前，完全没有任何办法来区分它们，这绝对是滋生各种bug的温床，因此`C++`标准委员会引入了这个新的关键字`typename`。

### 3. 使用`typename`的规则

+ `typename`在下列的情况中禁止使用：
  + 模板定义之外
  + 非限定类型
  + 基类列表中，如template <class T> class C1 : T::InnerType，不能再T::InnerType前面加typename
  + 构造函数的初始化列表中
+ 如果类型是依赖于模板参数的限定名，那么再它之前必须加`typename`，例如T::iterator（在基类列表或初始化列表中除外）
+ 其他情况下`typename`是可选的。

### 4. 会看问题的开始

```c++
typedef typename __type_traits<T>::has_trivial_destructor trivial_destructor;
```

现在这段代码就可以理解了，`has_trivial_destructor`是依赖于模板参数的限定名，因此必须在前面使用`typename`标识该限定名是类型，然后设置该类型的别名。