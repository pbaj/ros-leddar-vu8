cmake_minimum_required(VERSION 2.8.3)
project(leddar_vu8)

set(CMAKE_CXX_FLAGS "-std=c++11 -pthread")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  dynamic_reconfigure
  roslint
)

generate_dynamic_reconfigure_options(cfg/LeddarVu8.cfg)

catkin_package()

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(
    leddar_vu8
    src/main.cpp
    src/node.cpp
    src/leddar_vu8.cpp
)
add_dependencies(
    leddar_vu8
    ${PROJECT_NAME}_gencfg
)
target_link_libraries(
    leddar_vu8
    ${catkin_LIBRARIES}
    dl
)

roslint_cpp(
    src/main.cpp
    src/node.cpp
    src/node.h
    src/leddar_vu8.cpp
    src/leddar_vu8.h
)

install(FILES
    launch/leddar_vu8.launch
    launch/leddar_vu8.rviz
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
