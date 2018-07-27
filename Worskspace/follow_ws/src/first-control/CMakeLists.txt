cmake_minimum_required(VERSION 2.8.3)
project(first-control)

find_package(catkin REQUIRED COMPONENTS 
    roscpp actionlib move_base_msgs geometry_msgs naoqi_bridge_msgs)
find_package(Boost REQUIRED COMPONENTS system)

catkin_package( )

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(pub-square src/pub-square.cpp)
target_link_libraries(pub-square ${catkin_LIBRARIES})
add_dependencies(pub-square first-control_generate_messages_cpp)

add_executable(simple_navigation_goals src/simple_navigation_goals.cpp)
target_link_libraries(simple_navigation_goals ${catkin_LIBRARIES})

add_executable(mvt_arm src/mvt_arm.cpp)
target_link_libraries(mvt_arm ${catkin_LIBRARIES})

add_executable(grasping_test src/grasping_test.cpp)
target_link_libraries(grasping_test ${catkin_LIBRARIES})
