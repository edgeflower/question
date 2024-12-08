````cmake
insatll (TARGET(target)
动态库、静态库 
DESTINATION(destination)lib/${PROJECT_NAME}/)
````

````cmake
insatll (DIRECTORY(directory) 
config、 launch、 rviz   （等其他可执行文件所产生的文件）   
DESTINATION（destination)share/${PROJECT_NAME}/)
````



````cmake
install (DIRECTORY(directory）
可执行文件 
DESTINATION(destination)share/${PROJECT_NAME}/)
````



# 1. 详细分析每一部分install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

下面我们详细分析 `install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)` 的含义和每个部分的作用。这是一个典型的 CMake 安装命令，主要用于将指定目录的内容安装到目标目录中。

------

### 1. **`install`**

这是 CMake 的安装命令，负责定义项目构建后的安装规则。通过 `install()` 指定哪些文件、目录或目标需要被安装，以及它们的安装位置。

------

### 2. **`DIRECTORY launch`**

- **`DIRECTORY`**: 指定需要安装的目录。CMake会将该目录的内容递归复制到目标位置。

- **`launch`**: 表示需要安装的目录名称。在项目中，`launch` 目录通常包含配置文件或启动文件（如 ROS2 中的 `.launch.xml` 或 `.launch.py` 文件）。

  **作用**：

  - 递归地将 `launch` 目录及其子目录中的所有内容复制到指定的安装路径。

------

### 3. **`DESTINATION`**

`DESTINATION` 指定安装目标目录的路径。它是指在安装路径（通常是 `install` 或指定的 CMake 安装目录）中的相对路径。

------

### 4. **`share/${PROJECT_NAME}/`**

- **`share`**: 通常是 ROS2 包的安装约定，表示共享数据（如配置文件、启动文件等）的存放目录。

- **`${PROJECT_NAME}`**: 表示当前 CMake 项目的名称，通常通过 `project(<project_name>)` 定义。它会被替换为当前项目的名称。

  **完整路径**： 当你运行 `colcon build` 并安装后，`launch` 目录的内容将被复制到以下路径：

  ```
  <install_prefix>/share/<PROJECT_NAME>/launch/
  ```

  其中 `<install_prefix>` 是安装目录（如 ROS2 默认的 `install/` 目录）。

------

### 整体作用总结

这条指令会：

1. 将当前项目中的 `launch` 目录及其所有内容安装到路径 `install/share/<PROJECT_NAME>/launch/` 中。
2. 按照 ROS2 的文件组织结构，保证启动文件正确安装并可以被其他工具（如 `ros2 launch`）正常调用。

------

### 使用场景

- **ROS2 项目中**：

  - 这是一个典型的做法，用于安装 

    ```
    launch
    ```

     文件，使得你可以在安装包后，通过以下命令运行启动文件：

    ```bash
    ros2 launch <PROJECT_NAME> <launch_file>
    ```

  - 此时，`ros2` 会根据安装路径找到相关文件。

- **非ROS项目**： 如果不是 ROS2 项目，`install()` 同样可以用来复制配置文件、脚本或其他必要的资源文件到目标目录。

------

### 可选配置（常见补充）

可以在 `install()` 指令中增加额外选项，例如：

- **`FILES_MATCHING`**: 只安装匹配特定模式的文件，例如：

  ```cmake
  install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/ FILES_MATCHING PATTERN "*.launch.*")
  ```

  仅安装以 `.launch.xml` 或 `.launch.py` 结尾的文件。

- **`EXCLUDE`**: 排除特定文件或子目录，例如：

  ```cmake
  install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/ EXCLUDE PATTERN "*.tmp")
  ```

  排除所有临时文件（`.tmp`）。

------

如果还有其他细节需要补充或其他相关问题，欢迎随时提问！

# 2.install(TARGETS        pointlio_mapping        DESTINATION lib/${PROJECT_NAME}install(TARGETS        pointlio_mapping        DESTINATION lib/${PROJECT_NAME} 

这条 CMake 安装命令用于将目标文件（例如可执行文件或库文件）安装到指定目录中。我们来详细解析它的各个部分。

------

### 1. **`install`**

`install` 是 CMake 的安装命令，用于定义项目中生成的目标或文件的安装规则。它可以安装二进制文件、库、目录、配置文件等。

------

### 2. **`TARGETS`**

`TARGETS` 用于指定需要安装的目标（即由 CMake `add_library` 或 `add_executable` 创建的目标）。这些目标通常是：

- **可执行文件**：通过 `add_executable()` 定义的程序。
- **库文件**：通过 `add_library()` 定义的静态库或动态库。

在你的例子中，`pointlio_mapping` 是一个通过 `add_executable()` 或 `add_library()` 定义的目标名称。

------

### 3. **`DESTINATION`**

`DESTINATION` 指定目标文件在安装目录中的位置。这是一个相对路径，基于 CMake 安装的基础目录（通常是 `install/`）。

- **`lib/${PROJECT_NAME}`**：

  - **`lib`**: 表示目标文件将被安装到 `lib` 目录下，这是标准的库文件和部分二进制文件的安装位置。
  - **`${PROJECT_NAME}`**: 是当前 CMake 项目的名称（通过 `project(<project_name>)` 定义）。在安装时，`lib/` 目录下会创建一个子目录，以项目名称命名。

  **完整路径示例**： 如果安装目录是默认的 `install/` 且 `PROJECT_NAME` 为 `my_project`，最终安装路径会是：

  ```
  install/lib/my_project/
  ```

------

### 整体作用

这条命令的完整作用是：

- 将 `pointlio_mapping` 目标安装到路径 `install/lib/${PROJECT_NAME}/` 中。
- 这是典型的 CMake 安装实践，尤其是在 ROS2 中，符合其安装结构的约定。

------

### 使用场景

1. **ROS2 项目**：

   - 在 ROS2 项目中，库或工具程序通常安装到 

     ```
     lib/<PROJECT_NAME>
     ```

      目录，以便其他节点或工具可以调用。例如：

     - 动态库安装后可以被同项目中的其他组件加载。
     - 工具程序可以通过其完整路径执行。

2. **非 ROS2 项目**：

   - 在普通的 CMake 项目中，这种结构也常见，用于分类和组织多个目标，防止不同项目之间的文件混淆。

------

### 可选补充（常见用法）

1. **安装可执行文件**： 如果 `pointlio_mapping` 是一个可执行文件，可以直接安装到 `bin` 目录，例如：

   ```cmake
   install(TARGETS pointlio_mapping DESTINATION bin)
   ```

   这样，它会被安装到安装路径的 `bin/` 目录下。

2. **指定目标类型（可选）**： 你可以通过 `RUNTIME`, `LIBRARY`, `ARCHIVE` 等选项明确目标类型。例如：

   ```cmake
   install(TARGETS pointlio_mapping
           RUNTIME DESTINATION bin
           LIBRARY DESTINATION lib/${PROJECT_NAME}
           ARCHIVE DESTINATION lib/${PROJECT_NAME})
   ```

   这种方式明确指定了不同类型的目标文件的安装路径：

   - **`RUNTIME`**: 可执行文件。
   - **`LIBRARY`**: 动态库。
   - **`ARCHIVE`**: 静态库。

3. **指定安装规则**： 你可以为安装目标添加规则，例如：

   ```cmake
   install(TARGETS pointlio_mapping
           DESTINATION lib/${PROJECT_NAME}
           EXPORT my_project_targets)
   ```

   这会将目标文件的信息写入 `my_project_targets` 文件，用于其他 CMake 项目引用。

------

### 总结

```cmake
install(TARGETS
        pointlio_mapping
        DESTINATION lib/${PROJECT_NAME})
```

这条命令会将 `pointlio_mapping` 目标安装到 `install/lib/<PROJECT_NAME>/` 目录，适合用于安装动态库、静态库或工具程序，尤其是在 ROS2 或 CMake 项目中管理多个目标时。这种结构清晰，符合社区最佳实践。

如果有进一步的问题，欢迎继续提问！

# 3.install(        DIRECTORY config launch rviz_cfg        DESTINATION share/${PROJECT_NAME} )

我们来详细解析下面的 CMake 安装指令：

```cmake
install(
    DIRECTORY config launch rviz_cfg
    DESTINATION share/${PROJECT_NAME}
)
```

### 整体作用

这条指令用于将多个目录（`config`、`launch`、`rviz_cfg`）及其内容安装到目标路径 `share/${PROJECT_NAME}` 下。这种做法非常常见于 **ROS2** 项目中，用于安装配置文件、启动文件以及 RViz 可视化配置文件，确保在运行时能够正常找到这些资源。

------

### 分解分析

#### 1. **`install`**

这是 CMake 的安装命令，定义项目构建后需要安装的文件、目录或目标。

------

#### 2. **`DIRECTORY`**

- 表示需要安装的是一个或多个目录。
- 在本例中，`config`、`launch`、`rviz_cfg` 是需要安装的目录名称。
- **递归拷贝**：
  - CMake 默认会递归地将这些目录的内容及其子目录中的文件复制到目标位置。
  - 但它不会安装空目录（即，目录中没有任何文件时不会被复制）。

------

#### 3. **`DESTINATION share/${PROJECT_NAME}`**

- **`DESTINATION`** 指定目标安装路径。

- `share/${PROJECT_NAME}`

  ：

  - **`share`**: 按 ROS2 的文件组织结构，`share` 目录是共享文件（如配置文件、启动文件、RViz 文件等）的标准位置。

  - `${PROJECT_NAME}`

    : 当前项目的名称，通过 

    ```
    project(<project_name>)
    ```

     定义。例如，如果项目名称是 

    ```
    my_robot
    ```

    , 那么安装路径会是：

    ```
    <install_prefix>/share/my_robot/
    ```

- 完整安装路径示例

  ： 假设 CMake 安装路径是默认的 

  ```
  install/
  ```

  ，配置后的文件会被安装到：

  ```
  install/share/my_robot/config/
  install/share/my_robot/launch/
  install/share/my_robot/rviz_cfg/
  ```

------

#### 4. **被安装的内容**

1. **`config` 目录**：

   - 通常包含配置文件，例如：

     - YAML 文件（`*.yaml`）：用于定义参数。
     - JSON 文件（`*.json`）：用于描述环境或场景。

   - 示例内容：

     ```
     config/
     ├── params.yaml
     ├── robot_description.yaml
     ```

2. **`launch` 目录**：

   - 包含启动文件，用于启动 ROS2 节点或系统。

   - 常见文件格式：

     - `.launch.py` 或 `.launch.xml`。

   - 示例内容：

     ```
     launch/
     ├── robot_bringup.launch.py
     ├── simulation.launch.xml
     ```

3. **`rviz_cfg` 目录**：

   - 包含 RViz 可视化工具的配置文件。

   - 常见文件格式：

     - `.rviz`：描述 RViz 的布局、显示内容等。

   - 示例内容：

     ```
     rviz_cfg/
     ├── default_config.rviz
     ├── lidar_view.rviz
     ```

------

### 安装后如何使用

安装完成后，这些文件将被复制到安装目录中，便于运行时访问。例如：

1. **运行 Launch 文件**： 假设 `launch` 目录中有一个 `robot_bringup.launch.py` 文件，你可以通过以下命令运行它：

   ```bash
   ros2 launch my_robot robot_bringup.launch.py
   ```

2. **加载配置文件**： 在代码或启动文件中，可以引用 `config` 目录下的参数文件。例如：

   ```python
   Node(
       package='my_robot',
       executable='robot_node',
       name='robot_node',
       parameters=[os.path.join(get_package_share_directory('my_robot'), 'config', 'params.yaml')]
   )
   ```

3. **加载 RViz 配置**： 在 RViz 中，可以通过以下命令加载安装的 `.rviz` 文件：

   ```bash
   rviz2 -d install/share/my_robot/rviz_cfg/default_config.rviz
   ```

------

### 可选配置

#### 1. **过滤文件**

如果你只想安装某些类型的文件，可以使用 `FILES_MATCHING` 和 `PATTERN` 选项。例如：

```cmake
install(
    DIRECTORY config launch rviz_cfg
    DESTINATION share/${PROJECT_NAME}
    FILES_MATCHING PATTERN "*.yaml" PATTERN "*.launch.*" PATTERN "*.rviz"
)
```

- 只安装符合特定模式的文件，例如 YAML、launch 文件和 RViz 文件。

#### 2. **排除文件**

可以使用 `EXCLUDE` 选项排除特定文件或目录，例如：

```cmake
install(
    DIRECTORY config launch rviz_cfg
    DESTINATION share/${PROJECT_NAME}
    PATTERN "*.tmp" EXCLUDE
)
```

- 排除所有以 `.tmp` 结尾的临时文件。

------

### 总结

这条指令的作用是：

1. 安装 `config`、`launch` 和 `rviz_cfg` 目录到 `share/${PROJECT_NAME}` 中。
2. 符合 ROS2 的标准文件结构，便于运行时加载参数、启动文件和 RViz 配置。
3. 安装后，用户可以通过 `ros2 launch`、加载参数或使用 RViz 工具调用这些文件。

如果你有其他关于安装配置或 ROS2 文件组织的问题，欢迎随时提问！