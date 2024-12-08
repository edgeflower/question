# CMake

## 1编写一个简单的CMakeLists.txt

在终端写touch CMakeLists.txt

## 2. CMake 的使用

<font color='#f00'>CMake</font> 支持大写、小写、混写的命令。如果在编写<font color="#f00">CMakeLists.txt</font> 文件是使用的工具有对应的命令提示，那么大小写随缘即可，不用太过在意。

## 2.1 注释

### 2.1.1 注释行

<font color='#f9876789'>CMake</font> 使用`#`进`行注释`，可以放在任何位置。

``` 
# 这是一个CMakeLists.txt 文件
cmake_minimum_required(version 3.0.0)
```

### 2.1.2 注释块

`CMake` 使用`#[[]]`形式进行`块注释` 。

| #[[这是一个CMakeLists.txt文件 |
| ----------------------------- |
| 几个文件]]                    |

## 2.2 只有源文件

  ### 2.2.1 基础的三个命令

- `CMake_minimim_required` :指定使用cmake 的最低版本
  - ***可选，非必须，如果不加可能会有警告。***
- `project` : 定义工程名称，并可指定工程版本、工程描述、web网页地址、支持的语言（默认情况下支持所有语言），如果不需要这些都会是可以忽略的，只需指定出工程名字即可。
  - 实例

> #project 指令的语法
>
> project(<PROJECT-NAME>[<language-name>...])
>
> project(<PROJECT-NAME> 	//项目名字
>
> ​		[VERSION<major>[.<minor>[.[<patch>[.<tweak>]]]]  	//项目版本
>
> ​		[DESCRIPTION<project-description-string>	//项目描述]
>
> ​		[HOMEPACE_URL <url-string>]	//项目地址
>
> ​		[LANGUAGES<language-name>...])	//项目语言

- `add_executable`(可执行程序名、源文件名称)
- 会生成一个可执行程序
  - 这里的可执行文件名和`project`中的项目名没有任何关系
  - 原文件名可以是一个也可以是多个，如有多个可以用空格或`;`间隔。

```cmake
# 样式1
add_executable(app add.c div.c main.c mult.c sub.c)
# 样式2
add_executable(app add.c;div.c;main.c;mult.c;sub.c)
```

### 2.2.2 执行 CMake 命令

