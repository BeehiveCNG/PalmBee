cmake_minimum_required(VERSION 3.5)
project(px4_ros_com)

# Default to C99
if(NOT CMAKE_C_STANDARD)
	set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclpy REQUIRED)

#################
# Setup targets #
#################

include_directories(include)

# Add frame_transforms lib
add_library(frame_transforms SHARED src/lib/frame_transforms.cpp)
ament_target_dependencies(frame_transforms Eigen3 geometry_msgs sensor_msgs)
target_include_directories(frame_transforms PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
	$<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)


# examples/listeners/sensor_combined_listener
add_executable(sensor_combined_listener src/examples/listeners/sensor_combined_listener.cpp)
ament_target_dependencies(sensor_combined_listener rclcpp px4_msgs)
install(TARGETS sensor_combined_listener DESTINATION lib/${PROJECT_NAME})

# examples/listeners/vehicle_gps_position_listener
add_executable(vehicle_gps_position_listener src/examples/listeners/vehicle_gps_position_listener.cpp)
ament_target_dependencies(vehicle_gps_position_listener rclcpp px4_msgs)
install(TARGETS vehicle_gps_position_listener DESTINATION lib/${PROJECT_NAME})

# examples/advertisers/debug_vect_advertiser
add_executable(debug_vect_advertiser src/examples/advertisers/debug_vect_advertiser.cpp)
ament_target_dependencies(debug_vect_advertiser rclcpp px4_msgs)
install(TARGETS debug_vect_advertiser DESTINATION lib/${PROJECT_NAME})

# examples/offboard/offboard_control
add_executable(offboard_control src/examples/offboard/offboard_control.cpp)
ament_target_dependencies(offboard_control rclcpp px4_msgs)
install(TARGETS offboard_control DESTINATION lib/${PROJECT_NAME})

# examples/offboard/offboard_control_srv
add_executable(offboard_control_srv src/examples/offboard/offboard_control_srv.cpp)
ament_target_dependencies(offboard_control_srv rclcpp px4_msgs)
install(TARGETS offboard_control_srv DESTINATION lib/${PROJECT_NAME})

# examples/offboard/mission1
add_executable(mission1 src/examples/offboard/mission1.cpp)
ament_target_dependencies(mission1 rclcpp px4_msgs)
install(TARGETS mission1 DESTINATION lib/${PROJECT_NAME})

# examples/offboard/circle
add_executable(circle src/examples/offboard/circle.cpp)
find_package(geometry_msgs REQUIRED)
ament_target_dependencies(circle rclcpp px4_msgs geometry_msgs)
install(TARGETS circle DESTINATION lib/${PROJECT_NAME})

# examples/offboard/takeoff
add_executable(takeoff src/examples/offboard/takeoff.cpp)
ament_target_dependencies(takeoff rclcpp px4_msgs)
install(TARGETS takeoff DESTINATION lib/${PROJECT_NAME})

# examples/offboard/land
add_executable(land src/examples/offboard/land.cpp)
ament_target_dependencies(land rclcpp px4_msgs)
install(TARGETS land DESTINATION lib/${PROJECT_NAME})

# examples/offboard/landing
add_executable(landing src/examples/offboard/landing.cpp)
ament_target_dependencies(landing rclcpp px4_msgs)
install(TARGETS landing DESTINATION lib/${PROJECT_NAME})

# examples/offboard/yawing
add_executable(yawing src/examples/offboard/yawing.cpp)
ament_target_dependencies(yawing rclcpp px4_msgs)
install(TARGETS yawing DESTINATION lib/${PROJECT_NAME})

# examples/offboard/takeoffyawing
add_executable(takeoffyawing src/examples/offboard/takeoffyawing.cpp)
ament_target_dependencies(takeoffyawing rclcpp px4_msgs)
install(TARGETS takeoffyawing DESTINATION lib/${PROJECT_NAME})

# examples/offboard/forward
add_executable(forward src/examples/offboard/forward.cpp)
ament_target_dependencies(forward rclcpp px4_msgs)
install(TARGETS forward DESTINATION lib/${PROJECT_NAME})

############
# Install ##
############

# Export information to downstream packages
ament_export_dependencies(ament_cmake rclcpp rosidl_default_runtime eigen3_cmake_module Eigen3 px4_msgs geometry_msgs sensor_msgs)

ament_export_targets(export_frame_transforms HAS_LIBRARY_TARGET)

ament_export_include_directories(include)
ament_export_libraries(frame_transforms)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

install(TARGETS frame_transforms
	EXPORT export_frame_transforms
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
	INCLUDES DESTINATION include
)

# Install launch files.
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

# Install tests
install(DIRECTORY test DESTINATION share/${PROJECT_NAME}/)


############
# Testing ##
############

if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	# the following line skips the linter which checks for copyrights
	# uncomment the line when a copyright and license is not present in all source files
	#set(ament_cmake_copyright_FOUND TRUE)
	# the following line skips cpplint (only works in a git repo)
	# uncomment the line when this package is not in a git repo
	#set(ament_cmake_cpplint_FOUND TRUE)
	ament_lint_auto_find_test_dependencies()
endif()

###########
# Python ##
###########

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  src/examples/offboard_py/offboard_control.py
  src/examples/offboard_py/landing.py
  src/examples/offboard_py/fly.py
  src/examples/offboard_py/forward.py
  src/examples/offboard_py/vio_trf.py
  src/examples/offboard_py/setpoint.py
  src/examples/offboard_py/setpoint2.py
  src/examples/offboard_py/setpoint3.py
  src/examples/offboard_py/home_setpoint.py
  src/examples/offboard_py/going_home.py
  src/examples/offboard_py/forward_setpoint.py
  src/examples/offboard_py/control_data.py
  src/examples/offboard_py/control_data_circle.py
  src/examples/offboard_py/local_plan.py
  src/examples/offboard_py/local_plan_full.py
  #src/examples/offboard_py/mission1.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()