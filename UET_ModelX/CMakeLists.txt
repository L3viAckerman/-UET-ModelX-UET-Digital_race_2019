cmake_minimum_required(VERSION 2.8.3)
project(UET_ModelX)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cv_bridge
  image_transport
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
  DEPENDS system_lib
)

set(OpenCV_STATIC OFF)
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

set( OPENNI2_INCLUDE /home/nvidia/catkin_ws/src/astra_camera/OpenNI-Linux-Arm64-2.3.0.55/Include)
set( OPENNI2_REDIST  /home/nvidia/catkin_ws/src/astra_camera/OpenNI-Linux-Arm64-2.3.0.55/Redist)

include_directories( ${OPENNI2_INCLUDE} )
link_directories ( ${OPENNI2_REDIST} )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(
  include
   ${catkin_INCLUDE_DIRS} 
   )

add_library(${PROJECT_NAME}
   src/OpenNIHelper.cpp
   src/ros_handle.cpp
   src/lane_detect.cpp
   src/detecttrafficsign.cpp
   src/car_control.cpp
)

add_executable(${PROJECT_NAME}_node src/main.cpp)

add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(
  ${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}

  OpenNI2
)
