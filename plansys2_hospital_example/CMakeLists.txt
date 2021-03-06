cmake_minimum_required(VERSION 3.5)
project(plansys2_hospital_example)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_executor REQUIRED)


set(CMAKE_CXX_STANDARD 17)

set(dependencies
    rclcpp
    rclcpp_action
    plansys2_msgs
    plansys2_executor
)

add_executable(take_elevator_action_node src/take_elevator_action_node.cpp)
ament_target_dependencies(take_elevator_action_node ${dependencies})

add_executable(open_door_action_node src/open_door_action_node.cpp)
ament_target_dependencies(open_door_action_node ${dependencies})

add_executable(close_door_action_node src/close_door_action_node.cpp)
ament_target_dependencies(close_door_action_node ${dependencies})

add_executable(move_across_door_action_node src/move_across_door_action_node.cpp)
ament_target_dependencies(move_across_door_action_node ${dependencies})

add_executable(move_action_node src/move_action_node.cpp)
ament_target_dependencies(move_action_node ${dependencies})

add_executable(grab_object_action_node src/grab_object_action_node.cpp)
ament_target_dependencies(grab_object_action_node ${dependencies})

add_executable(release_object_action_node src/release_object_action_node.cpp)
ament_target_dependencies(release_object_action_node ${dependencies})

install(DIRECTORY launch pddl DESTINATION share/${PROJECT_NAME})

install(TARGETS
  take_elevator_action_node
  open_door_action_node
  close_door_action_node
  move_across_door_action_node
  move_action_node
  grab_object_action_node
  release_object_action_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()

#   # find_package(ament_cmake_gtest REQUIRED)
# endif()

ament_export_dependencies(${dependencies})

ament_package()