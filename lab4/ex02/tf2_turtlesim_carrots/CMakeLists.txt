cmake_minimum_required(VERSION 3.8)
project(tf2_turtlesim_carrots)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(turtlesim REQUIRED)

add_executable(turtle_broadcaster src/turtle_broadcaster.cpp)
add_executable(turtle_listener src/turtle_listener.cpp)
add_executable(carrot src/carrot_frame.cpp)

ament_target_dependencies(
  turtle_broadcaster
  geometry_msgs rclcpp tf2 tf2_ros turtlesim
)

ament_target_dependencies(
  turtle_listener
  geometry_msgs rclcpp tf2 tf2_ros turtlesim
)

ament_target_dependencies(
  carrot
  geometry_msgs rclcpp tf2_ros
)

install(TARGETS
  turtle_broadcaster
  turtle_listener
  carrot
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
