cmake_minimum_required(VERSION 3.8)
project(action_turtle_commands)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ExecuteTurtleCommands.action"
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

add_library(action_turtle_server SHARED src/action_turtle_server.cpp)
target_include_directories(action_turtle_server PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_compile_definitions(action_turtle_server PRIVATE "EXECUTE_ACTIONS_CPP_BUILDING_DLL")
ament_target_dependencies(action_turtle_server geometry_msgs rclcpp rclcpp_action rclcpp_components turtlesim)
rclcpp_components_register_node(action_turtle_server PLUGIN "action_turtle_executer::TurtleActionServer" EXECUTABLE execute_action_server)
target_link_libraries(action_turtle_server "${cpp_typesupport_target}")


add_library(action_turtle_client SHARED src/action_turtle_client.cpp)
target_include_directories(action_turtle_client PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_compile_definitions(action_turtle_client PRIVATE "EXECUTE_ACTIONS_CPP_BUILDING_DLL")
ament_target_dependencies(action_turtle_client geometry_msgs rclcpp rclcpp_action rclcpp_components turtlesim)
rclcpp_components_register_node(action_turtle_client PLUGIN "action_turtle_executer::TurtleActionClient" EXECUTABLE execute_action_client)
target_link_libraries(action_turtle_client "${cpp_typesupport_target}")


install(TARGETS
  action_turtle_client
  action_turtle_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
