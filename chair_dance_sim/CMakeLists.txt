cmake_minimum_required(VERSION 3.8)
project(chair_dance_sim)

find_package(ament_cmake REQUIRED)

# Install directories into the share folder
install(DIRECTORY
  urdf
  worlds
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
