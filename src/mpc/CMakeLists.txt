cmake_minimum_required(VERSION 3.22)
project(mpc)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 设置默认构建类型为 Release（如果未指定）
if(NOT CMAKE_BUILD_TYPE)
  # set(CMAKE_BUILD_TYPE Debug)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
# 强制启用 -O3 优化（覆盖 CMake 默认的 Release 标志）
if(CMAKE_BUILD_TYPE STREQUAL "Release")
  add_compile_options(-O3 -fPIC)
  # 可选：针对本地 CPU 优化
  # CMAKE_COMPILER_IS_GNUCXX已经废弃，改用 CMAKE_CXX_COMPILER_ID MATCHES "GNU"
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    # add_compile_options(-Wall -Wextra -Wpedantic)
    add_compile_options(-march=native)
  endif()
endif()
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g")
endif()

# 设置预编译指令，生成 compile_commands.json
# 然后让 c_cpp_properties.json 读取这个文件，防止预编译找不到cmake中定义的宏而报错
set(CMAKE_EXPORT_COMPILE_COMMANDS on)
set(PACKAGE_NAME ${PROJECT_NAME})
# set(CMAKE_CXX_STANDARD 11)


# find dependencies
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)



# 包含头文件目录
include_directories(
  src/config_reader
  src/utils
  src/main
)

# 添加子目录
add_subdirectory(src/config_reader)
add_subdirectory(src/utils)
add_subdirectory(src/main)

# Install other files.
install(DIRECTORY launch rviz config
  DESTINATION share/${PROJECT_NAME}/)


# 测试用
# if(BUILD_TESTING)
  # find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()
# endif()

###########
# Python ##
###########

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  src/vis/vis.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
