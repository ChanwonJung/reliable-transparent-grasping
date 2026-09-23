cmake_minimum_required(VERSION 3.8)
project(gen3_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf srdf launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
