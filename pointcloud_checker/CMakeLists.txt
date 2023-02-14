cmake_minimum_required(VERSION 3.5)
project(pointcloud_checker)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(pointcloud_checker_node SHARED
  src/pointcloud_checker_node.cpp
)

rclcpp_components_register_node(pointcloud_checker_node
  PLUGIN "pointcloud_checker::PointcloudCheckerNode"
  EXECUTABLE pointcloud_checker
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)
