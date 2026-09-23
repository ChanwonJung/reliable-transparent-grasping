cmake_minimum_required(VERSION 3.8)
project(gen3_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf srdf launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
