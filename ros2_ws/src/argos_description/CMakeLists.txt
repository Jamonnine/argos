cmake_minimum_required(VERSION 3.8)
project(argos_description)

find_package(ament_cmake REQUIRED)

# Install directories
install(DIRECTORY
  urdf
  launch
  config
  rviz
  worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
