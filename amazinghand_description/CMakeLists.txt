cmake_minimum_required(VERSION 3.14)
project(amazinghand_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf meshes rviz
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL  # meshes/rviz dirs are optional at scaffold stage
)

ament_package()
