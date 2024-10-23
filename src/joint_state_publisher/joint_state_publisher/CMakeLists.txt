cmake_minimum_required(VERSION 3.5)
project(joint_state_publisher)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY  test
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
