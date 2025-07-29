cmake_minimum_required(VERSION 3.5)
project(optitrack)

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
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Threads REQUIRED)
find_package(px4_msgs REQUIRED)

# Source files
set(VRPN_SOURCES
	src/publish_mocap_odometry.cpp
	src/VrpnClient.cpp
)


set(ESTIMATOR_RESET_SOURCES
	src/reset_estimator.cpp
)

set(VRPN_INSTALL_DIR "/usr/local/lib")

add_executable(publish_mocap_odometry_node ${VRPN_SOURCES})

ament_target_dependencies(publish_mocap_odometry_node
	rclcpp
	px4_msgs
	tf2_geometry_msgs
	tf2
)
target_include_directories(publish_mocap_odometry_node PRIVATE
	${CMAKE_SOURCE_DIR}/include/optitrack
	${VRPN_INSTALL_DIR}
)

target_link_libraries(publish_mocap_odometry_node
	${VRPN_INSTALL_DIR}/libvrpn.a
	${VRPN_INSTALL_DIR}/libquat.a
	Threads::Threads
)


add_executable(reset_estimator_node ${ESTIMATOR_RESET_SOURCES})

ament_target_dependencies(reset_estimator_node
	rclcpp
	px4_msgs
)

target_include_directories(reset_estimator_node PRIVATE
	${CMAKE_SOURCE_DIR}/include/optitrack
)


target_link_libraries(reset_estimator_node
	Threads::Threads
)

install(TARGETS
	publish_mocap_odometry_node	
	reset_estimator_node
	DESTINATION lib/${PROJECT_NAME}
)

ament_package()
