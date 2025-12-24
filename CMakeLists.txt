cmake_minimum_required(VERSION 3.5)
project(modulr_unitree_go2)

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
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(unitree_go REQUIRED)

find_package(PkgConfig)
pkg_check_modules(PC_ZeroMQ QUIET zmq)

# Find and build Unitree SDK
add_subdirectory(third_party/unitree_sdk2)

# Find and build messages from the Unitree ROS2 repo
file(GLOB_RECURSE UNITREE_MSGS "third_party/unitree_ros2/cyclonedds_ws/src/unitree/unitree_go/msg/*.msg")
set(UNITREE_MSGS_RELATIVE "")
foreach(file_path ${UNITREE_MSGS})
    file(RELATIVE_PATH rel_path "${CMAKE_CURRENT_SOURCE_DIR}" "${file_path}")
    list(APPEND UNITREE_MSGS_RELATIVE "${rel_path}")
endforeach()
rosidl_generate_interfaces(${PROJECT_NAME}
  ${UNITREE_MSGS_RELATIVE}
)

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
# rosidl_target_interfaces(fwd_motion
#   ${PROJECT_NAME} "rosidl_typesupport_cpp"
# )
install(TARGETS fwd_motion
  DESTINATION lib/${PROJECT_NAME})

# Camera Forwarding ROS 2 Node
add_executable(fwd_camera src/fwd_camera.cpp)
target_include_directories(fwd_camera PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(fwd_camera
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)
target_link_libraries(fwd_camera
  zmq
)

install(TARGETS fwd_camera
  DESTINATION lib/${PROJECT_NAME})

# Unitree Video interface to stream video over ZMQ

add_executable(unitree_video_zmq src/unitree_video_zmq.cpp)
ament_target_dependencies(unitree_video_zmq
  OpenCV
)
target_include_directories(unitree_video_zmq PRIVATE
  third_party/unitree_sdk2/include
  third_party/unitree_sdk2/thirdparty/include
  third_party/unitree_sdk2/thirdparty/include/ddscxx
)
target_link_libraries(unitree_video_zmq
  unitree_sdk2
  zmq
)
install(TARGETS unitree_video_zmq
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
