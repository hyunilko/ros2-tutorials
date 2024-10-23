cmake_minimum_required(VERSION 3.5)
project(adm_tf)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(urdf REQUIRED)

set(dependencies
  rclcpp
  tf2_ros
  geometry_msgs
  urdf
)

include_directories(
  include
  )

add_executable(adm_static_publisher_main src/adm_tf_static_publisher_main.cpp src/adm_tf_static_publisher_node.cpp)
ament_target_dependencies(adm_static_publisher_main ${dependencies})

add_executable(adm_listener_main src/adm_tf_listener_main.cpp src/adm_tf_listener_node.cpp)
ament_target_dependencies(adm_listener_main ${dependencies})


install(TARGETS adm_static_publisher_main adm_listener_main
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  config/morning_car.urdf
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()