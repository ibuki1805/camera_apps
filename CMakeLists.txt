cmake_minimum_required(VERSION 3.0.2)
project(camera_apps)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    std_msgs
    cv_bridge
    image_transport
    pluginlib
    nodelet
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES camera_apps
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ library
add_executable(object_detector_node src/object_detector_node.cpp src/object_detector.cpp)
target_link_libraries(
    object_detector_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    )

add_library(object_detector_nodelet src/object_detector.cpp src/object_detector_nodelet.cpp)

target_link_libraries(object_detector_nodelet
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

