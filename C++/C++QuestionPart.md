c++问题集

----



# 1.了解如何在类外访问 `numAndPerson` 这个成员。



### 1. 静态成员变量

在 `Person` 类中，我们定义了一个静态成员变量 `numAndPerson`，它用于存储参与者的信息。由于它是静态的，这意味着它属于类本身，而不是类的某个实例。这样，所有的 `Person` 对象都共享同一个 `numAndPerson`。

```cpp
private:
    static std::vector<std::pair<int, std::string>> numAndPerson;  // 声明为静态成员
```

### 2. 定义静态成员

在类外，我们需要提供静态成员的定义。这样做是为了在内存中分配实际的空间来存储 `numAndPerson` 的数据。

```cpp
std::vector<std::pair<int, std::string>> Person::numAndPerson = {
    {1000001, "小明"}, {1000002, "小红"}, {1000003, "小镧"}, {1000004, "小路"},
    {1000005, "小字"}, {1000006, "小排"}, {1000007, "小取"}, {1000008, "小空"},
    {1000009, "小私"}, {1000010, "小额"}, {1000011, "小哦"}, {1000012, "小清"}
};
```

这里我们初始化了 `numAndPerson`，使它包含一系列的参与者信息。

### 3. 公共方法获取参与者信息

我们提供了一个公共的静态方法 `getParticipants`，它返回 `numAndPerson` 的内容。这样，外部代码就可以调用这个方法来访问参与者信息。

```cpp
static std::vector<std::pair<int, std::string>> getParticipants() {
    return numAndPerson;  // 返回参与者信息
}
```

### 4. 打乱并显示参与者信息

在 `displayParticipants` 方法中，我们先随机打乱 `numAndPerson` 中的元素，然后将它们输出到控制台。

```cpp
static void displayParticipants() {
    std::random_device rd;
    std::mt19937 g(rd());

    std::shuffle(numAndPerson.begin(), numAndPerson.end(), g);
    for (const auto &item : numAndPerson) {
        std::cout << item << std::endl;
    }
}
```

### 5. 主函数中的调用

在 `main` 函数中，我们可以调用 `Person::displayParticipants()` 来显示参与者的信息，同时也可以调用 `Person::getParticipants()` 来获取参与者信息并在控制台打印出来。

```cpp
int main() {
    Person::displayParticipants();  // 显示随机打乱的参与者信息
    
    auto participants = Person::getParticipants();  // 获取参与者信息
    for (const auto& p : participants) {
        std::cout << p << std::endl;  // 打印所有参与者信息
    }

    return 0;
}
```

### 总结

通过使用静态成员变量和公共静态方法，我们可以在类外访问和操作 `numAndPerson`。这种设计使得代码更具封装性和模块性，同时避免了在外部直接操作内部数据的风险。希望这能帮助你更好地理解这个例子！如果还有其他问题，随时问我。





# 2.确保只输出一次打乱后的成员信息



，你可以在 `Person` 类中引入一个静态布尔变量来检查是否已经显示过。如果已经显示过，就不再重复输出。下面是更新后的代码示例：

```cpp
#pragma once
#include <iostream>
#include <string>
#include <vector>
#include <utility>
#include <algorithm>
#include <random>

template<typename T, typename U>
std::ostream & operator << (std::ostream &os, const std::pair<T, U> &pair) {
    os << "ID: " << pair.first << ", Name: " << pair.second;
    return os;
}

class Person {
public:
    static void displayParticipants() {
        // 检查是否已经显示过
        if (displayed) {
            std::cout << "参与者信息已显示过。" << std::endl;
            return;
        }

        // 随机数生成
        std::random_device rd;
        std::mt19937 g(rd());

        // 打乱参与者信息
        std::shuffle(numAndPerson.begin(), numAndPerson.end(), g);
        for (const auto &item : numAndPerson) {
            std::cout << item << std::endl;
        }

        // 设置标志为已显示
        displayed = true;
    }

    static std::vector<std::pair<int, std::string>> getParticipants() {
        return numAndPerson;  // 返回参与者信息
    }

private:
    static std::vector<std::pair<int, std::string>> numAndPerson;  // 声明为静态成员
    static bool displayed;  // 标志变量，记录是否已显示
};

// 在类外定义静态成员
std::vector<std::pair<int, std::string>> Person::numAndPerson = {
    {1000001, "小明"}, {1000002, "小红"}, {1000003, "小镧"}, {1000004, "小路"},
    {1000005, "小字"}, {1000006, "小排"}, {1000007, "小取"}, {1000008, "小空"},
    {1000009, "小私"}, {1000010, "小额"}, {1000011, "小哦"}, {1000012, "小清"}
};

bool Person::displayed = false;  // 初始化标志变量

// 在类外调用
int main() {
    Person::displayParticipants();  // 第一次显示参与者信息
    Person::displayParticipants();  // 再次调用，不会重复显示

    // 获取并打印参与者信息
    auto participants = Person::getParticipants();
    for (const auto& p : participants) {
        std::cout << p << std::endl;
    }

    return 0;
}
```









# 3.关键改动：

1. **添加静态布尔变量 `displayed`**：
   - 这个变量用于标记是否已经调用过 `displayParticipants` 方法。

2. **检查 `displayed` 的值**：
   - 在 `displayParticipants` 方法开始时检查 `displayed` 的值。如果为 `true`，则输出提示并返回，不再执行打乱和输出操作。

3. **更新 `displayed` 的值**：
   - 当参与者信息第一次成功显示后，将 `displayed` 设置为 `true`，确保后续调用不会再执行显示操作。

这样，无论你调用 `displayParticipants` 几次，输出结果都只会发生一次。

你的代码中定义了一个 `person` 类，里面有一个静态成员 `NumAndPerson` 用于存储人员信息，并且提供了方法来获取和显示这些信息。不过，代码中存在一些问题，特别是静态成员的定义和初始化。

### 修正和改进

1. **静态成员的定义**：
   - 你在 `person` 类内声明了 `NumAndPerson`，但在类外未定义它。你需要在源文件中提供它的定义。

2. **成员初始化**：
   - 你尝试用 `inline` 来定义 `NumAndPerson`，但是应该直接在类外定义，并且使用类名。

3. **函数声明**：
   - `test001` 函数被标记为 `inline`，这是不必要的，除非你打算在头文件中实现它。

### 更新后的代码示例

以下是修正后的代码示例：

**Person.h**

```cpp
#pragma once
#include <iostream>
#include <string>
#include <vector>
#include <utility>
#include <algorithm>
#include <random>

// 输出格式化操作符
template<typename T, typename U>
std::ostream & operator << (std::ostream &os, const std::pair<T, U> &pair) {
    os << "ID: " << pair.first << ", Name: " << pair.second;
    return os;
}

class person {
public:
    // 获取参与者信息
    static std::vector<std::pair<int, std::string>> getParticipants() {
        return NumAndPerson;
    }

    // 显示参与者信息
    static void DisplayParticipants() {
        // 随机数生成
        std::random_device rd;
        std::mt19937 g(rd());

        // 打乱参与者信息
        std::shuffle(NumAndPerson.begin(), NumAndPerson.end(), g);
        for (const auto &item : NumAndPerson) {
            std::cout << item << std::endl;
        }
    }

private:
    static std::vector<std::pair<int, std::string>> NumAndPerson; // 声明静态成员
};

// 静态成员的定义
std::vector<std::pair<int, std::string>> person::NumAndPerson = {
    {1000001, "小明"}, {1000002, "小红"}, {1000003, "小镧"}, {1000004, "小路"},
    {1000005, "小字"}, {1000006, "小排"}, {1000007, "小取"}, {1000008, "小空"},
    {1000009, "小私"}, {1000010, "小额"}, {1000011, "小哦"}, {1000012, "小清"}
};

// 测试函数
void test001() {
    person::DisplayParticipants();
}
```

**main.cpp**

```cpp
#include "Person.h"

int main() {
    test001();  // 调用测试函数
    return 0;
}
```

### 代码说明

1. **静态成员定义**：
   - 在 `Person.h` 文件中，`NumAndPerson` 的定义在类外完成，以确保链接器可以找到它。

2. **显示和获取方法**：
   - `DisplayParticipants` 随机打乱并显示参与者信息。
   - `getParticipants` 提供了获取静态成员的功能。

3. **测试函数**：
   - `test001` 函数简单调用 `DisplayParticipants`，可以在 `main` 函数中调用。

### 编译和运行

确保在同一目录下有 `Person.h` 和 `main.cpp` 文件，然后使用以下命令编译：

```bash
g++ main.cpp -o my_executable
```

运行生成的可执行文件：

```bash
./my_executable
```

这样，你就可以看到随机打乱后的参与者信息。如果有任何问题，随时告诉我！