cmake_minimum_required(VERSION 3.16)
project(piper_elevator_manipulation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_task_constructor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

add_executable(press_button_demo
  src/cartesian_line_planner.cpp
  src/contact_monitor.cpp
  src/five_dof_ik.cpp
  src/free_motion_route.cpp
  src/joint_state_cache.cpp
  src/press_executor.cpp
  src/press_parameters.cpp
  src/press_runner.cpp
  src/press_solution.cpp
  src/press_task_builder.cpp
  src/press_travel_monitor.cpp
  src/press_button_demo.cpp
  src/target_source.cpp
  src/trajectory_executor.cpp
)
target_include_directories(press_button_demo PRIVATE include)
ament_target_dependencies(press_button_demo
  geometry_msgs
  control_msgs
  moveit_core
  moveit_ros_planning
  moveit_task_constructor_core
  moveit_task_constructor_msgs
  rclcpp
  rclcpp_action
  sensor_msgs
  std_msgs
  tf2_eigen
  yaml_cpp_vendor
)

add_executable(virtual_contact_sim src/virtual_contact_sim.cpp)
ament_target_dependencies(virtual_contact_sim
  moveit_core
  moveit_ros_planning
  rclcpp
  sensor_msgs
  std_msgs
)

install(TARGETS press_button_demo virtual_contact_sim DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/go_home.py scripts/send_press_target.py
        scripts/preflight_check.py
        DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch docs config DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
