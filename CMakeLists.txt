cmake_minimum_required(VERSION 2.8.3)
project(darbot)

find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES darbot
#  CATKIN_DEPENDS other_catkin_pkg
)
include_directories(include ${catkin_INCLUDE_DIRS})

#When adding a new file, add an exectuable for it using the following syntax

#add_executable(FileName FileName.cpp)

#add_executable(GPIO_subscriber GPIO_subscriber.cpp)
add_executable(Darbot Darbot.cpp)
#add_executable(Ultrasonic Ultrasonic.cpp)
add_executable(Navigation Navigation.cpp)

#Next, add all of the libraries needed here. You should always include ${catkin_LIBRARIES). It is necessary for ROS to work. It also imports the libraries for basic packages such as std_msgs or geometry_msgs

#target_link_libraries(GPIO_subscriber pigpio)
#target_link_libraries(GPIO_subscriber ${catkin_LIBRARIES})

target_link_libraries(Darbot pigpio)
target_link_libraries(Darbot ${catkin_LIBRARIES})

#target_link_libraries(Ultrasonic pigpio)
#target_link_libraries(Ultrasonic ${catkin_LIBRARIES})

target_link_libraries(Navigation ${catkin_LIBRARIES})

target_compile_options(Darbot PRIVATE -std=c++11)
