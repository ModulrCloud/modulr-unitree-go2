cmake_minimum_required(VERSION 3.5)
project(modulr_unitree_go2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

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
find_package(geometry_msgs REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_api REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Find and build Unitree SDK
add_subdirectory(third_party/unitree_sdk2)
# Unitree Video interface to stream video over Zenoh
# Build zenoh-c from source for glibc compatibility (static linking)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "Build static zenoh-c library" FORCE)
add_subdirectory(third_party/zenoh-c)

# Motion Forwarding ROS 2 Node
add_executable(fwd_motion
  src/fwd_motion.cpp
  third_party/unitree_ros2/example/src/src/common/ros2_sport_client.cpp
)
target_include_directories(fwd_motion PRIVATE
  third_party/unitree_ros2/example/src/include
  third_party/unitree_ros2/example/src/include/common
  third_party/unitree_sdk2/include
  third_party/unitree_sdk2/thirdparty/include
  third_party/unitree_sdk2/thirdparty/include/ddscxx
)
ament_target_dependencies(fwd_motion
  rclcpp
  geometry_msgs
  unitree_api
)
install(TARGETS fwd_motion
  DESTINATION lib/${PROJECT_NAME})

# Odometry Bridge Node
add_executable(odom_bridge src/odom_bridge.cpp)
ament_target_dependencies(odom_bridge
  rclcpp
  nav_msgs
  tf2
  tf2_ros
  geometry_msgs
  unitree_go
)
install(TARGETS odom_bridge
  DESTINATION lib/${PROJECT_NAME})

add_executable(unitree_video src/unitree_video.cpp)
target_include_directories(unitree_video PRIVATE
  third_party/unitree_sdk2/include
  third_party/unitree_sdk2/thirdparty/include
  third_party/unitree_sdk2/thirdparty/include/ddscxx
)
target_link_libraries(unitree_video
  unitree_sdk2
  zenohc::static
)
install(TARGETS unitree_video
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
