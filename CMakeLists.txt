cmake_minimum_required(VERSION 3.5)
project(attention_l4ros2)

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
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(std_msgs REQUIRED)


set(CMAKE_CXX_STANDARD 17)

set(dependencies
    rclcpp
    rclcpp_action
    rclcpp_lifecycle
    trajectory_msgs
    std_msgs
)

include_directories(include)

install(DIRECTORY include/
  DESTINATION include/
)

add_library(${PROJECT_NAME} SHARED src/JointController.cpp)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(neck_controller_node src/neck_controller_node.cpp)
ament_target_dependencies(neck_controller_node ${dependencies})
target_link_libraries(neck_controller_node ${PROJECT_NAME})

install(DIRECTORY launch worlds DESTINATION share/${PROJECT_NAME})

install(TARGETS
  neck_controller_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_dependencies(${dependencies})

ament_package()
