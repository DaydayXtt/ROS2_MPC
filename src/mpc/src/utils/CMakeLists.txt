cmake_minimum_required(VERSION 3.22)
project(utils)

# 设置依赖，放到一个自定义变量中
set(UTILS_DEPENDS
  "Eigen3"
  "osqp"
  "OsqpEigen"
  "geometry_msgs"
  "nav_msgs"
)

# 添加动态库
add_library(utils SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/mpc_controller.cpp
)

set(utils_target
  utils
)

# 让动态库关联上依赖
ament_target_dependencies(${utils_target}
  ${UTILS_DEPENDS}
)

target_link_libraries(${utils_target}
  Eigen3::Eigen
  osqp::osqp
  OsqpEigen::OsqpEigen
)

# 安装动态库、节点和相关目录
# ament_export_targets(export_utils HAS_LIBRARY_TARGET)
# ament_export_libraries(${utils_target})
install(TARGETS
  ${utils_target}
  # EXPORT export_utils
  # ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  # RUNTIME DESTINATION bin
)

