cmake_minimum_required(VERSION 3.5)
project(optitrack)


# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
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

set(VRPN_INSTALL_DIR "/usr/local/lib")

# === VRPN Publisher Node ===

# Source files
set(VRPN_SOURCES
	src/publish_mocap_odometry.cpp
	src/VrpnClient.cpp
)


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

# === Reset Estimator Node ===

set(ESTIMATOR_RESET_SOURCES
	src/reset_estimator.cpp
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

# === Calculate Message Freq Node ===

set(CALC_FREQ_SOURCES
	src/calc_mocap_frequency.cpp
)

add_executable(calc_msg_frequency_node ${CALC_FREQ_SOURCES})

ament_target_dependencies(calc_msg_frequency_node
	rclcpp
)

target_include_directories(calc_msg_frequency_node PRIVATE
	${CMAKE_SOURCE_DIR}/include/optitrack
	${VRPN_INSTALL_DIR}
)

target_link_libraries(calc_msg_frequency_node
	${VRPN_INSTALL_DIR}/libvrpn.a
	${VRPN_INSTALL_DIR}/libquat.a
	Threads::Threads
)

# === Installation ===

install(TARGETS
	publish_mocap_odometry_node	
	reset_estimator_node
	calc_msg_frequency_node
	DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
	launch
	DESTINATION share/${PROJECT_NAME}
)

ament_package()
