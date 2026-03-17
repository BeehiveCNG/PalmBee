cmake_minimum_required(VERSION 3.8)
project(pb_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ===== Find dependencies =====
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)

# tf2 for quaternion / yaw
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ===== Executable =====
add_executable(control_manager src/control_manager.cpp)
add_executable(cm_setpoint src/cm_setpoint.cpp)

# ===== Link dependencies =====
ament_target_dependencies(control_manager
  rclcpp
  geometry_msgs
  nav_msgs
  mavros_msgs
  tf2
  tf2_geometry_msgs
)

ament_target_dependencies(cm_setpoint
  rclcpp
  geometry_msgs
  nav_msgs
  mavros_msgs
  tf2
  tf2_geometry_msgs
)

# ===== Install =====
install(TARGETS
  control_manager
  cm_setpoint
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
