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

set(CMAKE_BUILD_TYPE Debug)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_msgs REQUIRED)

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
    trajectory_msgs
    tf2_geometry_msgs
    std_msgs
    gazebo_msgs
    geometry_msgs
    ros2_knowledge_graph
)

include_directories(include)

install(DIRECTORY include/
  DESTINATION include/
)


add_library(lib_percep SHARED src/Perception.cpp ) # src/Perception.cpp )
ament_target_dependencies(lib_percep ${dependencies})

add_library(lib_joint SHARED src/JointController.cpp)
ament_target_dependencies(lib_joint ${dependencies})

add_executable(neck_controller_node src/neck_controller_node.cpp)
ament_target_dependencies(neck_controller_node ${dependencies})
target_link_libraries(neck_controller_node lib_joint)

add_library(lib_willing SHARED src/WillingToSeeSelector.cpp ) # src/WillingToSeeSelector.cpp )
ament_target_dependencies(lib_willing ${dependencies})

add_library(lib_focus_selector SHARED src/FocusSelector.cpp ) # src/FocusSelector.cpp )
ament_target_dependencies(lib_focus_selector ${dependencies})

add_executable(perception_node src/perception_node.cpp)
ament_target_dependencies(perception_node ${dependencies})
target_link_libraries(perception_node lib_percep)

add_executable(willing_to_see_node src/willing_to_see_node.cpp)
ament_target_dependencies(willing_to_see_node ${dependencies})
target_link_libraries(willing_to_see_node lib_willing )

add_executable(focus_selector_node src/focus_selector_node.cpp)
ament_target_dependencies(focus_selector_node ${dependencies})
target_link_libraries(focus_selector_node lib_focus_selector)

install(DIRECTORY launch worlds config DESTINATION share/${PROJECT_NAME})

install(TARGETS
  neck_controller_node
  perception_node
  willing_to_see_node
  focus_selector_node
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
