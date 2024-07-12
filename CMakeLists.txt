cmake_minimum_required(VERSION 3.5)
project(joint_states_subscriber_cpp)

# Default to C++14
set(CMAKE_CXX_STANDARD 14)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
include_directories(${trajectory_msgs_INCLUDE_DIRS})
include_directories(${sensor_msgs_INCLUDE_DIRS})
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

# Add executable
add_executable(joint_states_subscriber 
# src/joint_states_subscriber.cpp 
src/timer_publisher.cpp
# src/ecat_data_buffer.c 
# src/math_algorithm.cpp 
# src/pid_algorithm.cpp 
# src/filter_algorithm.cpp
)
ament_target_dependencies(joint_states_subscriber rclcpp rclcpp_components std_msgs)

# Link dependencies
target_link_libraries(joint_states_subscriber
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
)

# Install executable
install(TARGETS
  joint_states_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
