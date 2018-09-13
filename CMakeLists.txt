cmake_minimum_required(VERSION 2.8.3)
project(wp3_calibrator)

#set(CMAKE_MODULE_PATH /usr/local/lib/cmake)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} $ENV{HOME}/freenect2/lib/cmake/freenect2)

find_package(catkin REQUIRED COMPONENTS roscpp pcl_ros std_msgs sensor_msgs image_transport cv_bridge aruco)

find_package(OpenCV REQUIRED)
#find_package(PCL 1.8 REQUIRED COMPONENTS common io visualization filters keypoints features segmentation)
find_package(freenect2 REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

#include_directories(${PCL_INCLUDE_DIRS})
#INCLUDE_DIRECTORIES(${freenect2_INCLUDE_DIR})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
#  ${PCL_INCLUDE_DIRS}
  ${boost_INCLUDE_DIRS}
)

ADD_EXECUTABLE(calibrate src/calibrate.cpp)
ADD_DEPENDENCIES(calibrate ${catkin_EXPORTED_TARGETS})
TARGET_LINK_LIBRARIES(calibrate
   ${catkin_LIBRARIES} 
#   ${PCL_LIBRARIES} 
   ${OpenCV_LIBS} 
   ${freenect2_LIBRARIES}
)

#ADD_EXECUTABLE(calibrate_merge src/calibrate_merge.cpp)
#ADD_DEPENDENCIES(calibrate_merge ${catkin_EXPORTED_TARGETS})
#TARGET_LINK_LIBRARIES(calibrate_merge
#   ${catkin_LIBRARIES} 
##   ${PCL_LIBRARIES} 
#   ${OpenCV_LIBS} 
#   ${freenect2_LIBRARIES}
#)
