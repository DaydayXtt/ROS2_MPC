cmake_minimum_required(VERSION 3.22)
project(main)

# 设置依赖，放到一个自定义变量中
set(MAIN_DEPENDS
  "rclcpp"
  "Eigen3"
  "geometry_msgs"
  "nav_msgs"
)

# 添加节点
add_executable(main_node
  ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/mpc.cpp
)
set(main_target
  main_node
)

# 让动态库关联上依赖
ament_target_dependencies(${main_target}
  ${MAIN_DEPENDS}
)

# 链接库
target_link_libraries(${main_target}
  utils
  config_reader
)

# 安装节点
install(TARGETS
  ${main_target}
  DESTINATION lib/${PACKAGE_NAME} # 这里需要是外面的PACKAGE_NAME，如果是PROJECT_NAME，则会安装到main目录下
)
