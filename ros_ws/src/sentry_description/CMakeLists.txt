cmake_minimum_required(VERSION 3.8)
project(sentry_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config rviz urdf meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
