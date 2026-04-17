cmake_minimum_required(VERSION 3.8)
project(followingbot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

add_executable(car_control_node src/car_control.cpp)

ament_target_dependencies(car_control_node
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
)

install(TARGETS
  car_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
