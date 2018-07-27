cmake_minimum_required(VERSION 2.8.3)
project(pepper_tracking)

find_package(catkin REQUIRED COMPONENTS roscpp naoqi_bridge_msgs sensor_msgs std_msgs message_generation)
find_package(Boost REQUIRED COMPONENTS system)

 generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
 )

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(include ${catkin_INCLUDE_DIRS})

#add_executable(worker src/worker.cpp)
#target_link_libraries(worker ${catkin_LIBRARIES})

#add_executable(asker src/asker.cpp)
#target_link_libraries(asker ${catkin_LIBRARIES})
