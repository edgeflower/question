# 1.   例1

````cmake
cmake_minimum_required(VERSION 3.5)
project(cpp_pubsub)

#设置C++ 14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 添加编译选项，如果使用 GCC 或 Clang 编译器
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖项
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# 包含源代码并生成目标
add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)
# 安装目标
install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})
# 如果启用了测试，查找和配置测试工具
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # 以下行用于跳过检查版权的 linter（如果没有版权声明）
  # 如果你在项目中添加了版权声明，注释掉此行
  set(ament_cmake_copyright_FOUND TRUE)

  # 以下行用于跳过 cpplint 检查（只有在 git 仓库中有效）
  # 如果项目已经在 git 仓库中并且已经添加了版权声明，请注释掉此行
  set(ament_cmake_cpplint_FOUND TRUE)

  # 查找并配置测试依赖
  ament_lint_auto_find_test_dependencies()
endif()

# 调用 ament_package() 生成标准的 ROS 2 包
ament_package()
````

# 2.    例2

````cmake
# 设置CMake最低版本要求，并定义项目名称
cmake_minimum_required(VERSION 3.8)
project(more_interfaces)

# 编译器选项：如果使用GNU或Clang编译器，添加一些警告选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖项
find_package(ament_cmake REQUIRED)           # ament_cmake用于ROS 2的构建工具
find_package(rosidl_default_generators REQUIRED)  # 用于生成消息类型的代码
find_package(rclcpp REQUIRED)                # ROS 2的C++客户端库

# 指定要生成的消息文件
set(msg_files
 "msg/AddressBook.msg"
)

# 生成消息接口代码
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)

# 获取C++类型支持的目标名称，以便链接自定义消息类型
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

# 添加可执行文件并链接消息类型支持
add_executable(publish_address_book src/publish_address_book.cpp)
ament_target_dependencies(publish_address_book rclcpp)
target_link_libraries(publish_address_book "${cpp_typesupport_target}")

# 声明运行时依赖
ament_export_dependencies(rosidl_default_runtime)

# 安装生成的可执行文件到指定目录
install(TARGETS
  publish_address_book
  DESTINATION lib/${PROJECT_NAME}
)

# 测试配置：如果启用了测试，则加载和配置测试依赖项
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 以下两行用于跳过版权和代码风格检查
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 标记为ament_package，定义包的安装和导出行为
ament_package()

````

