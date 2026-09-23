cmake_minimum_required(VERSION 3.16)
project(piper_elevator_manipulation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_task_constructor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)

add_executable(press_button_demo
  src/five_dof_ik.cpp
  src/press_button_demo.cpp
)
target_include_directories(press_button_demo PRIVATE include)
ament_target_dependencies(press_button_demo
  geometry_msgs
  moveit_core
  moveit_ros_planning
  moveit_task_constructor_core
  moveit_task_constructor_msgs
  rclcpp
  sensor_msgs
  tf2_eigen
)

install(TARGETS press_button_demo DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch docs DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
