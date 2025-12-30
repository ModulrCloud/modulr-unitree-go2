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

# Find and build Unitree SDK
add_subdirectory(third_party/unitree_sdk2)
# Unitree Video interface to stream video over Zenoh
# Build zenoh-c from source for glibc compatibility (static linking)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "Build static zenoh-c library" FORCE)
add_subdirectory(third_party/zenoh-c)

# Motion Forwarding ROS 2 Node
add_executable(fwd_motion src/fwd_motion.cpp)
target_include_directories(fwd_motion PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(fwd_motion
  rclcpp
  geometry_msgs
  unitree_go
)
install(TARGETS fwd_motion
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
