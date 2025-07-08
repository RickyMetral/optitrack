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
find_package(Threads REQUIRED)
find_package(px4_msgs REQUIRED)

# Source files
set(SOURCES
	src/starling-pose-stream.cpp
	src/VrpnClient.cpp
)


set(VRPN_INSTALL_DIR "/usr/local/lib")

add_executable(publish_motive_odometry_node ${SOURCES})

ament_target_dependencies(publish_motive_odometry_node
	rclcpp
	px4_msgs
)

target_include_directories(publish_motive_odometry_node PRIVATE
	${CMAKE_SOURCE_DIR}/include/optitrack
	${VRPN_INSTALL_DIR}
	include
)

target_link_libraries(publish_motive_odometry_node
	${VRPN_INSTALL_DIR}/libvrpn.a
	${VRPN_INSTALL_DIR}/libquat.a
	Threads::Threads
)


get_target_property(INCLUDE_DIRS publish_motive_odometry_node  INCLUDE_DIRECTORIES)
message(STATUS "Include dirs: ${INCLUDE_DIRS}")


install(TARGETS
	publish_motive_odometry_node	
	DESTINATION lib/${PROJECT_NAME}
)



ament_package()
