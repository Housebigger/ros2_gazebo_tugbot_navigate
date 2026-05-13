cmake_minimum_required(VERSION 3.8)
project(tugbot_navigation)

set(Python3_EXECUTABLE /usr/bin/python3 CACHE FILEPATH "System python for ROS build" FORCE)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config worlds models maps rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
