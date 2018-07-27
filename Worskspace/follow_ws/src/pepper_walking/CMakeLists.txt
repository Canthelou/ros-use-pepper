cmake_minimum_required(VERSION 2.8.3)
project(pepper_walking)

find_package(catkin REQUIRED COMPONENTS roscpp naoqi_bridge_msgs sensor_msgs move_base_msgs geometry_msgs)
find_package(Boost REQUIRED COMPONENTS system)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(walking_sensor_return src/walking_sensor_return.cpp)
target_link_libraries(walking_sensor_return ${catkin_LIBRARIES})

add_executable(walking_autonomous src/walking_autonomous.cpp)
target_link_libraries(walking_autonomous ${catkin_LIBRARIES})
