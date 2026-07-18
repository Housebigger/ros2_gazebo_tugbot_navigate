cmake_minimum_required(VERSION 3.8)
project(tugbot_bringup)

set(Python3_EXECUTABLE /usr/bin/python3 CACHE FILEPATH "System python for ROS build" FORCE)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(ament_cmake REQUIRED)

foreach(dir launch config worlds models maps rviz urdf)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${dir}")
    install(DIRECTORY ${dir}
      DESTINATION share/${PROJECT_NAME}
    )
  endif()
endforeach()

install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
