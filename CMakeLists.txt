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
find_package(ament_index_cpp REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ros2_knowledge_graph REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(dependencies
    rclcpp
    rclcpp_action
    rclcpp_lifecycle
    gazebo_msgs
    geometry_msgs
    ros2_knowledge_graph
)

include_directories(include)

install(DIRECTORY include/
  DESTINATION include/
)

# add_library(${PROJECT_NAME} SHARED
#   src/Perception.cpp
#   # src/FocusSelector.cpp
# )
# ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(perception_node src/perception_node.cpp src/Perception.cpp)
ament_target_dependencies(perception_node ${dependencies})
# target_link_libraries(perception_node ${PROJECT_NAME})

# add_executable(focus_selector_node src/focus_selector_node.cpp)
# ament_target_dependencies(focus_selector_node ${dependencies})
# target_link_libraries(focus_selector_node ${PROJECT_NAME})

install(DIRECTORY launch worlds DESTINATION share/${PROJECT_NAME})

install(TARGETS
  perception_node
  # focus_selector_node
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
