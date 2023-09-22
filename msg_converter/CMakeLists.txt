cmake_minimum_required(VERSION 2.8)
project(msg_converter)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
    PCL REQUIRED
    roscpp
    rospy
    sensor_msgs
    std_msgs
    message_generation
    rosbag
    pcl_ros

    livox_ros_driver2       # livox custom message
)

## Generate messages in the 'msg' folder
add_message_files(FILES
    CustomPoint.msg
    CustomMsg.msg
)

catkin_package()

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)


## PCL library
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(converter 
    src/converter.cpp
)


target_link_libraries(converter ${LIVOX_LIDAR_SDK_LIBRARY} ${catkin_LIBRARIES} ${PCL_LIBRARIES} )
