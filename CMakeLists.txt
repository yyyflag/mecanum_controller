cmake_minimum_required(VERSION 3.8)
project(yahboomcar_mecanum_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Build controller library
add_library(mecanum_controller SHARED
  src/mecanum_controller.cpp
)
target_include_directories(mecanum_controller PRIVATE include)
ament_target_dependencies(mecanum_controller
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  geometry_msgs
)

# Install library
install(TARGETS mecanum_controller
  DESTINATION lib
)

# Install plugin description
install(FILES mecanum_controller_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
  launch urdf config
  DESTINATION share/${PROJECT_NAME}
)
# 安装 Python 脚本到 lib/${PROJECT_NAME}
install(
  PROGRAMS
    scripts/mecanum_kinematics_node.py
  DESTINATION
    lib/${PROJECT_NAME}
)
# Register plugin
pluginlib_export_plugin_description_file(controller_interface mecanum_controller_plugin.xml)

ament_package()