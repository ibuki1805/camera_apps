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
    camera_apps_msgs
    pcl_ros
    eigen_conversions
)
find_package(Eigen3 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES camera_apps
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
)

add_executable(object_detector_node src/object_detector_node.cpp src/object_detector.cpp)
add_dependencies(object_detector_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
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

add_executable(mask_rcnn_node src/mask_rcnn_node.cpp src/mask_rcnn.cpp)
add_dependencies(mask_rcnn_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(
    mask_rcnn_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    )

add_library(mask_rcnn_nodelet src/mask_rcnn.cpp src/mask_rcnn_nodelet.cpp)

target_link_libraries(mask_rcnn_nodelet
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

add_executable(object_state_estimator_node src/object_state_estimator_node.cpp src/object_state_estimator.cpp)
add_dependencies(object_state_estimator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(
    object_state_estimator_node
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    )

add_library(object_state_estimator_nodelet src/object_state_estimator.cpp src/object_state_estimator_nodelet.cpp)

target_link_libraries(object_state_estimator_nodelet
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

add_executable(motion_predictor_node src/motion_predictor_node.cpp src/motion_predictor.cpp)
add_dependencies(motion_predictor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(
    motion_predictor_node
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${Eigen3_LIBRARIES}
    )

add_library(motion_predictor_nodelet src/motion_predictor.cpp src/motion_predictor_nodelet.cpp)

target_link_libraries(motion_predictor_nodelet
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${Eigen3_LIBRARIES}
)

