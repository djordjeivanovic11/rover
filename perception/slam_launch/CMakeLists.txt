cmake_minimum_required(VERSION 3.5)
project(slam_launch)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
