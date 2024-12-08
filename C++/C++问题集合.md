# C++问题集合


## 1 声明、初始化、实例化的区别
- __声明是指使用一个对象的时候，先定义属于哪个类型，__比如string _str;_
  - str是声明（定义）了一个string类型的对象。

- __初始化可以理解为给声明的对象赋值的过程。__string _str ="hello";_
  - str现在已经被初始化了，实例化后它具有“hello”这个值。

- __实例化是类用到的，A a = new A();当new的时候为对象分配内存，__这个过程是对象的实例化。

  ### 1.1总结

  

- __声明，只生成对象不赋值的过程。__

- __初始化，是给对象赋值的过程。__

- __实例化，是使用new为对象分配内存的过程。__

## 2. vector<int>::iterator it

```c++
vector<int>::iterator  it = vect.begin();
ector<int> vect;
```



首先定义了一个int类型的向量；然后定义了一个具有int元素的迭代器类型。it的类型就是`vector<int>::iterator。`

### 2.1 <font color="#f00">vector</font>

vector不是一种数据类型，而只是一个类模板，可用来定义任意多种数据类型。vector类型的每一种都指定了其保存元素的类型。因此，vector<int>和vector <string>都是数据类型。


vector对象（以及其他标准库容器对象）的重要属性就在于可以在运行时高效地添加元素。因为vector增长的效率高，在元素值已知的情况下，最好是动态地添加元素。这种增长方式不同于C语言中的内置数据类型，也不同于大多数其他编程语言的数据类型。特别地，如果读者习惯了C或Java的风格，由于vector元素连续存储，可能希望最好是预先分配合适的空间。但事实上，为了达到连续性，C++的做法恰好相反。

虽然可以对给定元素个数的 vector 对象预先分配内存，但更有效的方法是先初始化一个空 vector 对象，然后再动态地增加元素。



### 2.2 vector迭代器

#### 2.2.1

​	除了使用下标来访问vector对象的元素外，标准库还提供了另一种检测元素的方法：**使用迭代器（iterator），**__它是一种允许程序员检查容器内元素，并实现元素遍历的数据类型。__

​	标准库为每一种标准容器（包括vector）定义了一种迭代器类型。迭代器类型提供了比下标操作更一般化的方法：所有的标准库容器都定义了相应的迭代器类型，而只有少数的容器支持下标操作。因为迭代器对所有的容器都适用，现代C++程序更倾向于使用迭代器而不是下标操作访问容器元素，即使对支持下标操作的vector类型也这样。

#### 2.2.2 容器的iterator类型

每种容器类型都定义了自己的迭代器类型，**vector：**

**vector<int>::iterator iter;**

​	这条语句定义了一个名为iter的变量，它的数据类型是由vector<int>定义的iterator类型。每个标准库容器类型都定义了一个名为iterator的成员，这里的iterator与迭代器实际类型的含义相同。

​	不同的容器类定义了自己的iterator类型，用于访问容器内的元素。换句话说，**每个容器定义了一种名为iterator的类型，而这种类型支持（概念上的）迭代器的各种行为。**

#### 2.2.3 begin和end操作

​	每种容器都定义了一对命名为begin和end的函数，用于返回迭代器。如果容器中有元素的话，由begin返回的迭代器指向第一个元素：

vector<int>::iterator iter = ivec.begin();

​	上述语句把iter初始化为由名为begin的vector操作返回的值。假设vector不空，初始化后，iter即指该元素为ivec[0]。

​	由end操作返回的迭代器指向vector的“末端元素的下一个”。通常称为超出末端迭代器(off-the-end iterator)，表明它指向了一个不存在的元素。如果vector为空，begin返回的迭代器与end返回的迭代器相同。

​	由end操作返回的迭代器并不指向vector中任何实际的元素，相反，它只是起一个哨兵（sentinel）的作用，表示我们已处理完vector中所有元素。

#### 2.2.4 vector迭代器的自增和解引用运算

​	迭代器类型定义了一些操作来获取迭代器所指向的元素，并允许程序员将迭代器从一个元素移动到另一个元素。

​	**迭代器类型可使用解引用操作符（*操作符）来访问迭代器所指向r 元素：**

***iter = 0;**

​	解引用操作符返回迭代器当前所指向的元素。假设iter指向vector对象ivec的第一个元素，那么*iter和ivec[0]就是指向同一个元素。上面这个语句的效果就是把这个元素的值赋为0。

​	迭代器使用自增操作符向前移动迭代器指向容器中下一个元素。从逻辑上说，迭代器的自增操作和int型对象的自增操作类似。对int对象来说，操作结果就是把int型值“加1”，而对迭代器对象则是把容器中的迭代器“向前移动一个位置”。因此，如果iter指向第一个元素，则++iter指向第二个元素。

​	由于end操作返回的迭代器不指向任何元素，因此不能对它进行解引用或自增操作。

#### 2.2.5  迭代器的其他运算

​	另一对可执行于迭代器的操作就是比较：用==或!=操作符来比较两个迭代器，如果两个迭代器对象指向同一个元素，则它们相等，否则就不相等。

##### 2.2.5.1 迭代器应用的程序示例

​	假设已声明了一个vector<int>型的ivec变量，要把它所有元素值重置为0，可以用下标操作来完成：



// equivalent loop using iterators to reset all the elements in ivec to 0

for (vector<int>::size_type ix = 0; ix != ivec.size(); ++ix)

       ivec[ix] = 0;

上述程序用for循环遍历ivec的元素，for循环定义了一个索引ix，每循环迭代一次ix就自增1。for循环体将ivec的每个元素赋值为0。

更典型的做法是用迭代器来编写循环：

// equivalent loop using iterators to reset all the elements in ivec to 0

for (vector<int>::iterator iter = ivec.begin();

                         iter != ivec.end(); ++iter)
    
    *iter = 0;  // set element to which iter refers to 0

​	for循环首先定义了iter，并将它初始化为指向ivec的第一个元素。for循环的条件测试iter是否与end操作返回的迭代器不等。每次迭代iter都自增1，这个for循环的效果是从ivec第一个元素开始，顺序处理vector中的每一元素。最后，iter将指向ivec中的最后一个元素，处理完最后一个元素后，iter再增加1，就会与end操作的返回值相等，在这种情况下，循环终止。

​	for循环体内的语句用解引用操作符来访问当前元素的值。和下标操作符一样，解引用操作符的返回值是一个左值，因此可以对它进行赋值来改变它的值。上述循环的效果就是把ivec中所有元素都赋值为0。

​	通过上述对代码的详细分析，可以看出这段程序与用下标操作符的版本达到相同的操作效果：从vector的第一个元素开始，把vector中每个元素都置为0。
​	如果vector为空，程序是安全的。如果ivec为空，则begin返回的迭代器不指向任何元素，由于没有元素，所以它不能指向任何元素——在这种情况下，从begin操作返回的迭代器与从end操作返回的迭代器的值相同，因此for语句中的测试条件立即失败。

#### 2.2.6 const_iterator

​	前面的程序用vector::iterator改变vector中的元素值。每种容器类型还定义了一种名为const_iterator的类型，该类型只能访问容器内元素，但不能改变其值。

​	当我们对普通iterator类型解引用时，得到对某个元素的非const引用。而如果我们对const_iterator类型解引用时，则可以得到一个指向const对象的引用，如同任何常量一样，该对象不能进行重写。

​	例如，如果text是vector<string>类型，程序员想要遍历它，输出每个元素,可以这样编写程序：

// use const_iterator because we won't change the elements

for (vector<string>::const_iterator iter = text.begin();

                                    iter != text.end(); ++iter)
    
    cout << *iter << endl; // print each element in text

​	除了是从迭代器读取元素值而不是对它进行赋值之外，这个循环与前一个相似。由于这里只需要借助迭代器进行读，不需要写，这里把iter定义为const_iterator类型。当对const_iterator类型解引用时，返回的是一个const值。不允许用const_iterator进行赋值：

​	**不要把const_iterator对象与const的iterator对象混淆起来。声明一个const迭代器时，必须初始化迭代器。一旦被初始化后，就不能改变它的值：**

## 3. C/C++中string和int相互转换的常用方法

### 3.1 通过 std::to_string() 函数转换

```	c++
#include <iostream>

int main()
{
    int num = 123;
    std::cout << std::to_string(num);
    return 0;
}

```

**这种方式在 C++11 中才能使用，编译时记得加上 `--std=c++11` 的选项**

### 3.2 通过 sprintf 转换

```c++
#include <stdio.h>

int main()
{
    int num = 123;
    char buffer[256];
    sprintf(buffer, "%d", num);

    printf("%s", buffer);
    return 0;
}

```

**这是一种C语言中的转换方式，`sprintf` 也可以换成更安全的 `snprintf` 函数**

## 4.C/C++语言中erase（）函数的用法



erase函数的原型如下：

1. string& erase ( size_t pos = 0, size_t n = npos );
2. iterator erase ( iterator position）;
3. iterator erase ( iterator first, iterator last );

也就是说有三种用法：

**（1）erase(pos,n); 删除从pos开始的n个字符，比如erase(0,1)就是删除第一个字符**

**（2）erase(position);删除position处的一个字符(position是个string类型的迭代器)**

**（3）erase(first,last);删除从first到last之间的字符（first和last都是迭代器）**

```c++
#include<string>
#include<iostream>
using namespace std;
int main ()
{

string str ("This is an example phrase.");

string::iterator it;

//第（1）种方法

str.erase (10,8);

cout << str << endl;        // "This is an phrase."

//第（2）种方法

it=str.begin()+9;

str.erase (it);

cout << str << endl;        // "This is a phrase."

//第（3）种方法

str.erase (str.begin()+5, str.end()-7);

cout << str << endl;        // "This phrase."

return 0;

}
```

----



----

## 5.栈

<https://zhuanlan.zhihu.com/p/656750303>

​	**数据结构栈（Stack）是一种基本的线性数据结构，它按照后进先出（LIFO，Last In First Out）的原则存储和操作数据。在栈中，数据项的插入和删除操作只能在栈顶进行，因此栈的结构特性与操作方式相对简单，但却具有广泛的应用场景。**

## 6.在C++中不能使用 类型 string 的左值 初始化 phone 类型的成员 'PName'

这个错误提示通常意味着你在使用 `string` 类型的左值来初始化一个类的成员时，出现了类型不匹配或转换的问题。为了更具体地理解这个问题，我们可以考虑以下几种可能的场景。

**常见错误原因分析**

1. **成员类型不匹配**：例如，如果你在类中定义了一个 `PName` 成员，但它的类型不是 `std::string`，而是某种自定义类型或者其他非兼容类型，而你尝试用 `string` 来初始化它，那么就会出现这种类型不匹配的问题。
2. **没有定义适当的构造函数**：如果你有一个自定义类型 `phone`，而且它的成员 `PName` 是该类型的对象，但 `phone` 类中没有合适的构造函数或转换操作来接受 `string` 类型的输入，也可能导致这种错误。
3. **未包含正确的头文件**：如果你忘记包含 `#include <string>`，那么编译器可能无法正确识别 `std::string` 类型。

**示例1：成员类型不匹配**

````c++
class phone {
public:
    // 假设 PName 是一个整型变量，而我们试图用 string 初始化它
    int PName; 

    phone(int name) : PName(name) {}
};

int main() {
    std::string name = "iPhone";  // 使用 string 类型的变量
    phone p(name);  // 这里会报错，无法将 string 传给 int
    return 0;
}

````

​	在这个例子中，`PName` 是 `int` 类型，而你尝试使用 `std::string` 来初始化它，这是不兼容的，会导致编译错误。

**解决办法**

​	要解决这个问题，可以确保类的成员类型与初始化时的数据类型兼容，或者提供必要的转换。以下是如何修改上面的例子：

#### 解决方案1：更改成员变量的类型

```c++
#include <iostream>
#include <string>

class phone {
public:
    std::string PName;  // 将 PName 改为 string 类型

    phone(std::string name) : PName(name) {}  // 正确的构造函数
};

int main() {
    std::string name = "iPhone";
    phone p(name);  // 现在不会报错
    std::cout << "Phone name: " << p.PName << std::endl;
    return 0;
}

```

#### 解决方案2：使用转换构造函数

如果你希望保留 `PName` 的类型，并能够使用 `std::string` 初始化，可以为 `phone` 类定义一个转换构造函数或重载赋值运算符：

```
cpp复制代码#include <iostream>
#include <string>

class phone {
public:
    int PName;  // 假设 PName 仍然是 int 类型

    // 使用构造函数将 string 转换为 int（例如字符串长度）
    phone(std::string name) : PName(name.length()) {}
};

int main() {
    std::string name = "iPhone";
    phone p(name);  // 允许使用 string 来初始化
    std::cout << "Phone name length: " << p.PName << std::endl;  // 输出字符串长度
    return 0;
}
```

在这个示例中，我们使用字符串的长度来初始化 `PName`，虽然这是一个简单的示例，但它展示了如何通过构造函数进行转换。

### 示例2：忘记包含头文件

另一个常见的错误是忘记包含必要的头文件：

```
cpp复制代码// 忘记包含 <string>
class phone {
public:
    std::string PName;  // 这里可能会导致未定义的错误
};
```

确保你在文件顶部包含了：

```
cpp


复制代码
#include <string>
```

### 总结

- 确保类的成员变量类型与初始化值类型匹配。
- 如需使用不同类型进行初始化，请定义适当的转换构造函数或方法。
- 确保包含了正确的头文件，例如 `#include <string>`。

如果你提供的代码更具体，错误的根源就能更精确定位。

## 7.解释在C++中的类对象作为类成员

在C++中，类对象作为类成员是指将一个类的对象作为另一个类的成员变量。这种设计方式允许一个类包含另一个类的实例，以便组合类的功能或表示更复杂的关系。这种关系通常称为**复合关系**（composition）。类对象作为成员的好处在于能够更好地组织代码，使得类之间可以有逻辑上的层次结构。

### 具体例子

```
cpp复制代码#include <iostream>
using namespace std;

// 定义一个类 Address 表示地址
class Address {
public:
    string city;
    string street;
    int number;

    Address(string c, string s, int n) : city(c), street(s), number(n) {}

    void showAddress() const {
        cout << "Address: " << street << ", " << city << ", " << number << endl;
    }
};

// 定义一个类 Person 表示人，Person 类中包含 Address 类对象作为成员
class Person {
private:
    string name;
    int age;
    Address address;  // Address 对象作为类成员

public:
    // 构造函数
    Person(string n, int a, Address addr) : name(n), age(a), address(addr) {}

    void showInfo() const {
        cout << "Name: " << name << ", Age: " << age << endl;
        address.showAddress();  // 调用 Address 对象的方法
    }
};

int main() {
    // 创建 Address 对象
    Address addr("New York", "5th Ave", 100);
    
    // 创建 Person 对象，并将 Address 对象作为参数传入
    Person person("John Doe", 30, addr);
    
    // 输出 Person 信息
    person.showInfo();

    return 0;
}
```

### 解释

1. **Address 类**：表示一个地址，包含城市、街道、门牌号，并提供一个显示地址的方法 `showAddress()`。
2. **Person 类**：表示一个人，包含姓名、年龄和一个 `Address` 对象。`Address` 对象在 `Person` 类中作为成员变量出现。
3. **Person 构造函数**：通过构造函数初始化 `name`、`age` 和 `address`，其中 `address` 是通过 `Address` 对象传入的。
4. 在 `main()` 函数中，首先创建一个 `Address` 对象，然后将其作为参数传入 `Person` 对象的构造函数，最后调用 `showInfo()` 方法，显示 `Person` 和 `Address` 的相关信息。

### 注意事项

- **成员对象的构造顺序**：类中的成员对象（如 `address`）会在类的构造函数体执行前初始化，因此在类的初始化列表中需要对它进行初始化。
- **内存管理**：当 `Person` 对象被销毁时，其成员 `Address` 对象也会被自动销毁，无需手动管理。

### 这种设计的应用场景

1. **复合类型的构建**：例如，一个`Car`类可以有一个`Engine`对象作为其成员，因为一辆车包含一个发动机。
2. **代码重用**：通过类成员对象可以重用其他类的代码，避免重复实现功能。

## 8.  如果不用引用的方式返回，相当于返回与p2不同的另一个Person（只是age都是20），那么后续的加年龄操作与p2就没有关系了 //黑马程序员

**原因**

<https://blog.csdn.net/weixin_46624734/article/details/119611319>



___



## 9.在C++中创建头文件后怎么在终端中是头文件添加CMake



在C++项目中使用CMake进行构建时，你需要在CMakeLists.txt文件中指定你的源文件和头文件。以下是在CMake中添加头文件的一般步骤：

1. **创建CMakeLists.txt文件**：如果你的项目中还没有CMakeLists.txt文件，你需要创建一个。这个文件位于项目的根目录下。

2. **添加头文件**：在CMakeLists.txt文件中，你可以使用`target_include_directories`命令来添加头文件的路径。这允许编译器在编译时找到这些头文件。

3. **指定源文件**：使用`add_executable`或`add_library`命令来指定你的源文件（.cpp文件），这样CMake就知道哪些文件需要被编译。

下面是一个简单的CMakeLists.txt文件的例子，它展示了如何添加头文件：

```cmake
cmake_minimum_required(VERSION 3.10)

# 定义项目名称和语言
project(MyProject LANGUAGES CXX)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 添加可执行文件
add_executable(MyExecutable main.cpp)

# 添加头文件的路径
target_include_directories(MyExecutable PRIVATE ${PROJECT_SOURCE_DIR}/include)

# 如果你有多个源文件，可以这样添加
# add_executable(MyExecutable main.cpp file1.cpp file2.cpp)

# 如果你有库文件，可以这样添加
# add_library(MyLibrary STATIC lib1.cpp lib2.cpp)
# target_include_directories(MyLibrary PRIVATE ${PROJECT_SOURCE_DIR}/include)
```

在这个例子中，`${PROJECT_SOURCE_DIR}/include`是头文件的路径，你需要根据你的项目结构来调整这个路径。`PRIVATE`关键字表示这些头文件只对`MyExecutable`目标有效。

4. **运行CMake**：在终端中，进入到包含CMakeLists.txt的目录，然后运行以下命令：

```bash
cmake .
```

这将生成Makefile或其他构建系统文件。

5. **构建项目**：使用生成的Makefile构建项目：

```bash
make
```

或者，如果你在使用其他构建系统（如Ninja），使用相应的命令。

请注意，头文件本身不需要被编译，但编译器需要知道它们的位置以便在编译过程中找到它们。这就是为什么我们使用`target_include_directories`来指定头文件路径的原因。

## 10. 友元

### 10.1友元



在C++中，友元（friend）是一种特殊的类成员，它允许非成员函数访问类的私有（private）和保护（protected）成员。友元函数或友元类不是类的成员，但它们可以访问类的内部数据和成员函数，就像它们是类的成员一样。

友元的声明通常在类定义的末尾，使用`friend`关键字。下面是友元的一些用途和特点：

1. **访问私有成员**：友元函数可以访问类的私有和保护成员，而普通函数则不能。

2. **非成员函数**：友元可以是非成员函数，这意味着它们不属于任何类。

3. **友元类**：一个类可以声明另一个类为其友元，这样后者就可以访问前者的所有私有和保护成员。

4. **控制访问**：友元提供了一种方式来控制哪些函数可以访问类的内部数据，即使这些函数不是类的成员。

5. **灵活性**：友元提供了一种灵活性，允许在类之间共享数据，而不需要将所有数据公开。

下面是一个简单的友元函数的例子：

```cpp
#include <iostream>
using namespace std;

class Box {
private:
    double width;
    double height;
    double depth;

public:
    Box(double w, double h, double d) : width(w), height(h), depth(d) {}

    // 声明友元函数
    friend double volume(Box& b);
};

// 友元函数的实现
double volume(Box& b) {
    return b.width * b.height * b.depth;
}

int main() {
    Box box(3.3, 1.2, 1.5);
    cout << "The volume of the box is: " << volume(box) << endl;
    return 0;
}
```

在这个例子中，`volume`函数是一个友元函数，它可以访问`Box`类的私有成员`width`、`height`和`depth`来计算体积。

需要注意的是，友元机制破坏了类的封装性，因为它允许外部函数访问类的私有成员。因此，在使用友元时应该谨慎，确保不会滥用这种访问权限。

### 10.2🤔 友元函数和成员函数有什么区别？

友元函数和成员函数在C++中都是类的一部分，但它们之间有几个关键的区别：

1. **定义位置**：
   - **成员函数**：定义在类的内部或外部，但声明在类的内部。
   - **友元函数**：定义在类的外部，声明在类的内部，使用`friend`关键字。

2. **访问权限**：
   - **成员函数**：可以直接访问类的公有（public）、保护（protected）和私有（private）成员。
   - **友元函数**：可以访问类的公有、保护和私有成员，但它们不是类的成员，因此不能直接访问类的成员。

3. **类成员资格**：
   - **成员函数**：是类的成员，可以被类的实例调用。
   - **友元函数**：不是类的成员，不能被类的实例直接调用，需要通过类的对象或指针/引用来调用。

4. **调用方式**：
   - **成员函数**：可以通过对象实例（`obj.func()`）或指针（`obj->func()`）调用。
   - **友元函数**：需要通过对象实例（`obj.func()`）或指针（`obj->func()`）来调用，但它们本身不是对象的一部分。

5. **封装性**：
   - **成员函数**：是类封装的一部分，有助于保护类的内部状态。
   - **友元函数**：破坏了类的封装性，因为它们可以访问类的私有成员。

6. **继承**：
   - **成员函数**：可以被继承，子类可以重写（override）父类的成员函数。
   - **友元函数**：不是类的成员，因此不能被继承，也不能被重写。

7. **多态性**：
   - **成员函数**：可以是虚函数（virtual），支持多态性。
   - **友元函数**：不能是虚函数，不支持多态性。

8. **构造函数和析构函数**：
   - **成员函数**：可以是构造函数或析构函数，用于初始化和清理对象。
   - **友元函数**：不能是构造函数或析构函数。

9. **静态成员**：
   - **成员函数**：可以是静态成员函数，不依赖于类的实例。
   - **友元函数**：不能是静态成员函数，因为它们不是类的成员。

10. **重载**：
    - **成员函数**：可以被重载，只要参数列表不同。
    - **友元函数**：不能被重载，因为它们不是类的成员。

总结来说，成员函数是类的一部分，可以直接访问类的成员，并且受到类的封装和继承机制的保护。友元函数虽然可以访问类的私有成员，但它们不是类的成员，不能被继承或重写，也不能直接作为对象的一部分被调用。

### 10.3 成员函数作友元

在C++中，不仅可以将非成员函数声明为友元函数，也可以将一个类的成员函数声明为另一个类的友元。这种做法允许一个类的成员函数访问另一个类的私有（private）和保护（protected）成员。这在某些设计场景中非常有用，比如当你想要实现一些特定的类之间的紧密协作时。

#### 10.3.1成员函数作为友元的声明

要将一个类的成员函数声明为另一个类的友元，你需要在类定义中使用`friend`关键字。下面是一个简单的例子来说明这个过程：

```cpp
#include <iostream>
using namespace std;

// 声明类A
class A {
private:
    int value;

public:
    A(int val) : value(val) {}

    // 声明B的成员函数为友元
    friend void func(A& a, int x);
};

// 声明类B
class B {
public:
    void func(A& a, int x) {
        a.value = x; // 直接访问A的私有成员
    }
};

int main() {
    A a(10);
    B b;
    b.func(a, 20); // 通过B的成员函数修改A的私有成员

    cout << "Value of A: " << a.value << endl; // 输出: Value of A: 20
    return 0;
}
```

在这个例子中：
- 类`A`有一个私有成员`value`。
- 类`B`的成员函数`func`被声明为类`A`的友元。这意味着`func`可以访问`A`的私有成员。
- 在`main`函数中，我们创建了`A`和`B`的实例，并通过`B`的`func`成员函数修改了`A`的私有成员`value`。

#### 10.3.2 注意事项

虽然将成员函数声明为友元可以提供灵活性，但它也有一些潜在的问题：
- **破坏封装性**：友元函数可以访问类的私有成员，这可能会破坏类的封装性，使得类的内部状态更容易被外部修改。
- **增加复杂性**：友元关系增加了代码的复杂性，因为它涉及到类之间的更深层次的耦合。
- **维护困难**：随着项目的扩展，维护这些友元关系可能会变得更加困难，特别是当涉及到多个类和多个函数时。

因此，在使用友元函数时，应该谨慎考虑是否真的需要这种级别的访问权限，以及是否有其他设计方法可以避免使用友元。

## 11. 汉字"你"的长度是多少

### 在 UTF-8 编码中，一个汉字通常占用三个字节。

因此，汉字“你”在 UTF-8 编码中的长度是 3。如果是在 UTF-16 或 UTF-32 编码中，一个汉字通常占用两个字节或四个字节。在不同的编码系统中，字符所占的字节数可能会有所不同。在 Unicode 标准中，汉字“你”的码点是 U+4F60。

## 12.throw是什么

`throw` 是 C++ 中用于抛出异常的关键字。异常处理是一种错误管理机制，当程序中发生不可恢复的错误或异常情况时，可以通过抛出异常来中断正常的执行流，并将控制权转移给一个专门的“异常处理器”。

### `throw` 的用法

在 C++ 中，`throw` 语句用于抛出一个异常。例如：

```cpp
throw std::runtime_error("An error occurred");
```

这里，`throw` 抛出了一个 `std::runtime_error` 类型的异常，并将消息 `"An error occurred"` 传递给它。这个异常会中断程序的执行，并试图找到一个“捕获”它的地方（即 `try-catch` 块）。

### 异常处理机制 (`try-catch`)

C++ 使用 `try` 和 `catch` 关键字来捕获异常。通过这种方式，可以在程序中指定一些代码来“捕获”抛出的异常，并处理它们。例如：

```cpp
try {
    // 可能发生异常的代码
    throw std::runtime_error("Something went wrong");
} catch (const std::runtime_error& e) {
    // 处理异常
    std::cout << "Caught an exception: " << e.what() << std::endl;
}
```

- `try` 块中的代码会尝试执行，如果发生异常，会跳过剩余部分并进入 `catch` 块。
- `catch` 块会接收异常对象并执行相应的代码。在这里，`e.what()` 返回异常的描述信息。

### 在 `PriorityQueue` 代码中的作用

在 `PriorityQueue` 类中，如果在 `get()` 或 `peek()` 方法中调用时队列为空，就会 `throw` 一个 `std::runtime_error` 异常：

```cpp
if (empty()) {
    throw std::runtime_error("PriorityQueue is empty");
}
```

这意味着调用这些方法的代码需要处理异常，否则程序将会崩溃。

## 13. random_shuffile 

关于 `random_shuffle` 的问题，以下是一些关键信息：

### 1.函数定义和用法：

`std::random_shuffle` 是 C++ 标准库中的一个函数，用于随机打乱一个序列中的元素。它定义在 `<algorithm>` 头文件中，有两个版本：

- 一个接受两个参数：要打乱的元素范围的开始和结束迭代器。
- 另一个接受三个参数：除了范围的开始和结束迭代器外，还有一个随机数生成器。

### 2.随机数生成器：

- 第一个版本的 `random_shuffle` 使用默认的随机数生成器，即 `std::rand()` 函数。
- 第二个版本允许用户传入自定义的随机数生成器，该生成器需要接受一个整数参数，并返回区间 [0, N) 中的随机数。

### 3.C++14 和 C++17 中的变化：

- `std::random_shuffle` 在 C++14 中已经被标记为不推荐使用（deprecated），在 C++17 中被完全移除。
- 推荐使用的替代函数是 `std::shuffle`，它接受一个均匀随机数生成器作为参数，提供更好的随机性。

### 4.随机数生成器的选择：

- 在使用 `std::random_shuffle` 时，如果不手动设置随机数种子，每次同一时间洗同一副牌，得到的结果也是相同的。这是因为 `std::rand()` 函数依赖于全局状态，如果不设置种子，它将产生相同的随机序列。

### 5.C++11 中的随机数工具：

- C++11 提供了 `<random>` 头文件，其中包含多个随机数生成工具，包括均匀随机位生成器（URBGs）和随机数分布器。这些工具可以用来生成各种概率分布的随机数。

### 7.`std::shuffle` 的使用：

- `std::shuffle` 是 C++11 标准添加的，用于随机打乱序列的元素。它的第三个参数需要传递一个均匀随机数生成器对象。

### 8.实现细节：

- `std::random_shuffle` 的实现不是由标准规定，所以即使使用相同的随机数生成器或随机函数，不同标准库实现的结果也可能不同。

总结来说，`std::random_shuffle` 由于其随机性不足和已被新的标准弃用，不建议在新的 C++ 代码中使用。取而代之的是 `std::shuffle`，它提供了更好的随机性和灵活性。





## 14.make_pair

在 C++ 中，`std::make_pair` 是一个非常有用的函数模板，它用于创建 `std::pair` 对象。这个函数接受两个参数，并返回一个包含这两个参数的 `std::pair` 对象。这使得创建包含两个相关值的 `pair` 对象变得更加方便。

以下是 `std::make_pair` 的基本用法：

```cpp
#include <utility> // 包含 std::make_pair
#include <iostream>

int main() {
    // 创建一个 pair 对象，包含两个 int
    std::pair<int, int> pair1 = std::make_pair(1, 2);

    // 创建一个 pair 对象，包含一个 int 和一个 string
    std::pair<int, std::string> pair2 = std::make_pair(1, "one");

    // 输出 pair 对象的内容
    std::cout << "Pair 1: (" << pair1.first << ", " << pair1.second << ")\n";
    std::cout << "Pair 2: (" << pair2.first << ", " << pair2.second << ")\n";

    return 0;
}
```

输出将会是：

```
Pair 1: (1, 2)
Pair 2: (1, one)
```

`std::make_pair` 是一个模板函数，所以它可以用于创建任何类型的 `pair` 对象，只要这些类型可以被拷贝构造。它的语法简单，使得代码更加清晰和易于理解。

在 C++11 之后，你也可以使用列表初始化（list initialization）和花括号 `{}` 来创建 `pair` 对象，这在某些情况下可以替代 `std::make_pair`：

```cpp
std::pair<int, std::string> pair3{1, "one"};
// 或者
std::pair<int, std::string> pair4 = {1, "one"};
```

这两种方法创建的 `pair` 对象与 `std::make_pair` 创建的完全相同。使用列表初始化的方式在现代 C++ 代码中越来越流行，因为它更简洁，而且可以很好地与统一初始化一起工作。