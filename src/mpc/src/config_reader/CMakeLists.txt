cmake_minimum_required(VERSION 3.22)
project(config_reader)

# 设置依赖，放到一个自定义变量中
set(CONFIG_DEPENDS
  "rclcpp"
  "yaml-cpp"
  "ament_index_cpp"
)

# 添加动态库
add_library(config_reader SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/config_reader.cpp
)

set(config_target
  config_reader
)

# 让动态库关联上依赖
ament_target_dependencies(${config_target}
  ${CONFIG_DEPENDS}
)

target_link_libraries(${config_target}
  yaml-cpp
)

# 安装动态库、节点和相关目录
install(TARGETS
  ${config_target}
  LIBRARY DESTINATION lib
)

