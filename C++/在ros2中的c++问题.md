# 1.`class FibonacciActionServer : public rclcpp::Node`里的public的作用

明白了！在 `class FibonacciActionServer : public rclcpp::Node` 里，`public` 是用来指定 **继承方式** 的。

### `public` 继承的作用

在 C++ 中，类的继承有三种方式：`public`、`protected` 和 `private`。这三种继承方式会影响基类（父类）成员在派生类（子类）中的访问权限。

当我们写 `class FibonacciActionServer : public rclcpp::Node` 时，这表示 **`FibonacciActionServer` 以 `public` 的方式继承了 `rclcpp::Node`**。具体含义如下：

1. **保持基类的 `public` 成员在子类中仍然是 `public` 的**：
   - 例如，`rclcpp::Node` 的 `get_logger()` 是 `public` 成员，`FibonacciActionServer` 继承它时依然是 `public` 的，这样可以直接在 `FibonacciActionServer` 中使用 `get_logger()`。

2. **保持基类的 `protected` 成员在子类中仍然是 `protected` 的**：
   - 如果 `rclcpp::Node` 中有 `protected` 成员，那么在 `FibonacciActionServer` 中这些成员仍然是 `protected`，只能在类内部或派生类中使用。

### 为什么用 `public` 继承？

`public` 继承是最常用的继承方式，因为它保持了基类接口的访问权限，遵循了“**is-a**”关系：即 `FibonacciActionServer` 是一种 `rclcpp::Node`。这样一来，`FibonacciActionServer` 类可以被当作 `rclcpp::Node` 来使用，也能访问 `rclcpp::Node` 提供的所有 `public` 接口。

### 总结

在 `class FibonacciActionServer : public rclcpp::Node` 中，`public` 继承的目的是让 `FibonacciActionServer` 完全继承 `rclcpp::Node` 的 `public` 和 `protected` 接口，这样在外部可以像访问 `rclcpp::Node` 一样访问 `FibonacciActionServer`。！





# 2. using Fibonacci = action_tutorials_interfaces::action::Fibonacci中using是做什么的

在这行代码中：

```cpp
using Fibonacci = action_tutorials_interfaces::action::Fibonacci;
```

**`using`** 是 C++ 中定义类型别名的一种方式，类似于 `typedef`，但语法更简单直观。

### 详细解释

1. **类型别名**：
   - 这行代码定义了一个新的名称 `Fibonacci`，将它作为 `action_tutorials_interfaces::action::Fibonacci` 的别名。
   - 这样一来，之后在代码中可以直接用 `Fibonacci` 来表示 `action_tutorials_interfaces::action::Fibonacci`。

2. **简化代码**：
   - `action_tutorials_interfaces::action::Fibonacci` 是一个较长的名称，每次使用它都很麻烦。通过 `using` 创建别名，可以简化代码，提高可读性。
   - 这样在类的成员函数里，直接写 `Fibonacci`，使代码更加清晰简洁。

### 示例

在使用别名后，这段代码中的 `Fibonacci` 就等价于 `action_tutorials_interfaces::action::Fibonacci`：

```cpp
using Fibonacci = action_tutorials_interfaces::action::Fibonacci;

Fibonacci fibonacci_goal;  // 等价于 action_tutorials_interfaces::action::Fibonacci fibonacci_goal;
```

### 总结

`using` 关键字在这里的作用是定义一个类型别名，将复杂或冗长的类型名称简化为更简洁的名称，以便在代码中更方便地使用。





# 3.`action_tutorials_interfaces::action::Fibonacci`是action_tutorials_interfaces作用域里action作用域里的Fibonacci吗

是的，`action_tutorials_interfaces::action::Fibonacci` 确实表示 **`action_tutorials_interfaces` 作用域里 `action` 作用域里的 `Fibonacci`**。

在这行代码中：

```cpp
using Fibonacci = action_tutorials_interfaces::action::Fibonacci;
```

- `action_tutorials_interfaces` 是最外层的命名空间。
- `action` 是 `action_tutorials_interfaces` 命名空间内的子命名空间。
- `Fibonacci` 是 `action` 命名空间下定义的一个类或类型。

### 作用域的分层理解

1. **最外层命名空间 `action_tutorials_interfaces`**：
   - 这是一个顶层命名空间，通常是为一个特定的包或模块创建的，以避免名字冲突。
   
2. **子命名空间 `action`**：
   - `action` 命名空间进一步将类型组织在一个更具体的层级内。在 ROS 2 中，`action` 命名空间通常专门用于动作接口，以便清晰地区分出动作相关的类型。

3. **类型 `Fibonacci`**：
   - `Fibonacci` 是在 `action` 子命名空间中定义的动作接口类型，表示特定的动作结构或类（比如目标、反馈、结果等）。

所以，当使用 `action_tutorials_interfaces::action::Fibonacci` 时，代码是在访问 `action_tutorials_interfaces` 命名空间下 `action` 子命名空间中的 `Fibonacci` 类型。





# 4. `using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>`;中<>是做什么的

在这行代码中：

```cpp
using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;
```

`<>` 是用于模板类的参数列表，它指定了 **模板类型参数**。让我们来逐步解析这行代码。

### 解释

1. **`rclcpp_action::ServerGoalHandle<Fibonacci>`**：
   - `rclcpp_action::ServerGoalHandle` 是一个 **模板类**，它需要一个类型作为模板参数。
   - `Fibonacci` 是作为模板参数传递给 `ServerGoalHandle` 类的类型。

2. **模板类（`ServerGoalHandle`）**：
   - `ServerGoalHandle` 是一个模板类，它定义了如何处理某个特定动作（在这里是 Fibonacci 动作）的目标（Goal）。
   - 模板类的作用是使得一个类可以根据不同的类型参数进行定制。在这里，`ServerGoalHandle` 处理的目标类型就是 `Fibonacci`。
   
3. **`<Fibonacci>`**：
   - `<Fibonacci>` 是指定模板类型参数的地方，这里我们传递了 `Fibonacci` 类型，告诉 `ServerGoalHandle` 类它需要操作的是 `Fibonacci` 类型的目标。
   
4. **`using GoalHandleFibonacci = ...`**：
   - `using` 是 C++11 引入的一个别名声明，它用于创建类型别名。这里创建了一个名为 `GoalHandleFibonacci` 的别名，表示 `rclcpp_action::ServerGoalHandle<Fibonacci>` 类型。
   - 这样，你就可以使用 `GoalHandleFibonacci` 来代替长且复杂的类型名 `rclcpp_action::ServerGoalHandle<Fibonacci>`，使代码更简洁可读。

### 总结

- `rclcpp_action::ServerGoalHandle<Fibonacci>` 是一个模板类，`<>` 中的 `Fibonacci` 是模板参数，告诉 `ServerGoalHandle` 这个类将用于处理 `Fibonacci` 类型的目标。
- 通过 `using GoalHandleFibonacci = ...`，你为 `rclcpp_action::ServerGoalHandle<Fibonacci>` 创建了一个更简短的别名 `GoalHandleFibonacci`，使得后续代码中可以用这个别名来代替原本较长的类型名。





# 5. 下面我们来深入剖析这一行代码：

```cpp
this->action_server_ = rclcpp_action::create_server<Fibonacci>(
  this,
  "fibonacci",  // 动作的名字
  std::bind(&FibonacciActionServer::handle_goal, this, _1, _2), // 目标处理函数
  std::bind(&FibonacciActionServer::handle_cancel, this, _1), // 取消处理函数
  std::bind(&FibonacciActionServer::handle_accepted, this, _1)); // 接受请求的处理函数
```

### 1. **`rclcpp_action::create_server<Fibonacci>`**

- `rclcpp_action::create_server` 是 ROS 2 提供的一个函数，用于创建一个动作服务器。
- 这个函数是模板函数，模板类型 `<Fibonacci>` 表示我们要创建的动作服务器处理的是 `Fibonacci` 类型的动作，也就是在这个服务器上处理斐波那契数列相关的目标请求。

   **动作的核心**包括三部分：
   - **Goal**：目标，客户端请求要执行的任务。
   - **Feedback**：反馈，服务器在执行任务时向客户端报告当前的进度。
   - **Result**：结果，任务完成后，服务器向客户端返回最终结果。

   `Fibonacci` 就是定义了这些内容的 ROS 2 动作类型（在 `action_tutorials_interfaces` 包中定义）。该动作包括了斐波那契数列的目标（`order`）、反馈（部分数列）、和最终结果（完整数列）。

### 2. **`this`**

- `this` 是一个指向当前类 `FibonacciActionServer` 的指针。
- 它告诉 `create_server` 函数，这个动作服务器属于当前的 `FibonacciActionServer` 类实例。
- 这意味着，创建的动作服务器将会与当前类的成员函数（如 `handle_goal`、`handle_cancel`、`handle_accepted`）绑定，执行目标时需要使用这些函数来处理客户端的请求。

### 3. **`"fibonacci"`**

- `"fibonacci"` 是该动作服务器的名称，用于标识这个动作。在 ROS 2 中，每个动作服务器都需要一个名字，客户端会通过这个名字来发送请求。
- 这个名字是动作服务器与 ROS 2 系统交互的标识符。

### 4. **`std::bind(&FibonacciActionServer::handle_goal, this, _1, _2)`**

- `std::bind` 是 C++ 标准库的一个功能，它可以将成员函数绑定到一个具体的对象（此处是 `FibonacciActionServer`），并返回一个可调用的函数对象。
- `&FibonacciActionServer::handle_goal` 是一个指向成员函数的指针，表示 `handle_goal` 这个函数是用于处理接收到的目标请求的。
- `this` 是类实例的指针，表示调用该函数时，成员函数的 `this` 指针将指向当前对象。
- `_1` 和 `_2` 是 `std::bind` 的占位符，表示将来调用时会传入的参数。
  - `_1` 对应函数 `handle_goal` 的第一个参数：`const rclcpp_action::GoalUUID & uuid`。
  - `_2` 对应第二个参数：`std::shared_ptr<const Fibonacci::Goal> goal`。

  具体来说，`handle_goal` 函数的定义是：
  ```cpp
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Fibonacci::Goal> goal)
  ```
  - `uuid` 是目标的唯一标识符，用于跟踪目标。
  - `goal` 是客户端请求的目标数据，包含了要计算的斐波那契数列的项数。

  `std::bind(&FibonacciActionServer::handle_goal, this, _1, _2)` 将目标请求的处理逻辑与实际目标数据绑定起来，确保当目标请求到来时，能够调用该函数并正确处理。

### 5. **`std::bind(&FibonacciActionServer::handle_cancel, this, _1)`**

- `std::bind(&FibonacciActionServer::handle_cancel, this, _1)` 用来绑定处理取消请求的函数 `handle_cancel`。
- `handle_cancel` 函数定义如下：
  ```cpp
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  ```
  - `goal_handle` 是目标句柄，包含了与目标相关的信息，用于取消或管理目标。
  
  通过 `std::bind`，当客户端请求取消目标时，会调用 `handle_cancel` 函数。

### 6. **`std::bind(&FibonacciActionServer::handle_accepted, this, _1)`**

- `std::bind(&FibonacciActionServer::handle_accepted, this, _1)` 用来绑定目标被接受后的处理函数 `handle_accepted`。
- `handle_accepted` 函数定义如下：
  ```cpp
  void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  ```
  - `goal_handle` 是一个指向目标句柄的共享指针，包含了与当前目标相关的所有信息。
  
  当目标被接受后，`handle_accepted` 会被调用，在该函数内，你可以启动一个新的线程来执行目标，避免阻塞主线程。

### 7. **`this->action_server_`**

- `this->action_server_` 是当前类 `FibonacciActionServer` 的一个成员变量，用来存储创建的动作服务器实例。
- `action_server_` 是类型为 `rclcpp_action::Server<Fibonacci>::SharedPtr` 的智能指针，它管理着创建的动作服务器对象。
- `action_server_` 会被用于在后续的代码中管理这个动作服务器的生命周期和行为。

### 总结

这一行代码实际上是在设置和创建一个 ROS 2 动作服务器，具体的功能和结构包括：

- 创建一个名为 `"fibonacci"` 的动作服务器。
- 将目标请求、取消请求、目标接受后的处理分别与类的成员函数 `handle_goal`、`handle_cancel`、`handle_accepted` 绑定。
- 创建的动作服务器会处理客户端发送的关于斐波那契数列的目标请求，并在接收到请求时执行相应的函数来处理目标、取消和反馈。

通过 `std::bind`，这些函数就可以和 ROS 2 动作服务器框架交互，使得你的代码能够响应并处理来自客户端的动作请求。 

# 6.`rclcpp_action::GoalResponse handle_goal`

`( const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Fibonacci::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}`