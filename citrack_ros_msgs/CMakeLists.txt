cmake_minimum_required(VERSION 3.8)
project(citrack_ros_msgs)

# Compile as C++11
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(action_msgs REQUIRED)


# Set the path to the ROS 2 workspace
# set(ROS2_WS_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/..)  # Adjust the path accordingly

# Add the link directories for external libraries
# link_directories("${ROS2_WS_SRC_DIR}/uwb_ros_msgs")
# link_directories("${ROS2_WS_SRC_DIR}/citrack_vicon/DataStreamSDK_10.1/")


# Set message files
set(msg_files
  "msg/ViconPosition.msg"
  "msg/UwbAnchor.msg"
  "msg/UwbTag.msg"
  "msg/UwbTags.msg"
  "msg/UwbRange.msg"
  "msg/DarknetBoundingBox.msg"
  "msg/DarknetBoundingBoxes.msg"
  "msg/DarknetObjectCount.msg"
  "msg/RvizBirdsEyeParam.msg"
  "msg/CustomTag.msg"
  "msg/MultiTags.msg"
)

# Set action files
set(action_files
  "action/DarknetCheckForObjects.action"
)

# Generate messages and actions
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${action_files}
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs sensor_msgs action_msgs
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
