cmake_minimum_required(VERSION 2.8.3)
project(rosovbc)


#set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -g -Wall") # For G++ <4.7; also appears to be supported in G++ 4.7+ as alias to c++11

## Find catkin and any catkin packages
#find_package(catkin REQUIRED COMPONENTS move_base_msgs roscpp std_msgs genmsg

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs genmsg
message_filters
sensor_msgs
cv_bridge
roscpp
std_msgs
image_transport
visp_bridge
tf
actionlib
naoqi_bridge_msgs
)

find_package(VISP REQUIRED)
add_definitions(${VISP_DEFINITIONS})

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS} ${UEYE_INCLUDE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/include/rosovbc
${CMAKE_CURRENT_SOURCE_DIR}/include/rosovbc/photometricVS/)

MESSAGE("##### " ${CMAKE_CURRENT_SOURCE_DIR})
include_directories(SYSTEM
#  ${VISP_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(
 #/home/yalj/catkin_ws/devel/lib
# ${VISP_LIBRARY_DIRS}
  ${catkin_LIBRARY_DIRS}
  ~/catkin_ws/devel/lib/lib64
)


#link_directories(
#/home/yalj/catkin_ws/devel/lib
#${VISP_LIBRARY_DIRS}
#${catkin_LIBRARY_DIRS}
#)

#catkin_package()

#add_executable(photometric_visual_servo src/main_photo_visual_servo.cpp src/PhotoVisualServo.cpp src/VisualServoTools.cpp)
#target_link_libraries(photometric_visual_servo ${catkin_LIBRARIES} PhotoVS)
#add_dependencies(photometric_visual_servo rosovbc_generate_messages_cpp)

SET(SRC
./src/photometricVS/CFeatureLuminanceOmni.cpp
./src/photometricVS/CFeatureLuminanceOmni.h
./src/photometricVS/CFeatureLuminanceOmniFeature.cpp
./src/photometricVS/CFeatureLuminanceOmniFeatureDepth.cpp
./src/photometricVS/CFeatureLuminanceOmniFeaturesd.cpp
./src/photometricVS/CFeatureLuminanceOmniFeaturesdDepth.cpp
./src/photometricVS/CFeatureLuminanceOmniInteractionMatrix.cpp
./src/photometricVS/CFeatureLuminanceOmniNeighborhoods.cpp
./src/photometricVS/CImageFilter.cpp
./src/photometricVS/CImageTools.cpp
./src/camera/CCameraOmniParameters.cpp
./src/camera/CCameraOmniParameters.h
./src/camera/CModel.cpp
./src/camera/CModel.h
./src/camera/commun.h
./src/camera/COmni.cpp
./src/camera/COmni.h
./src/camera/CPerspective.cpp
./src/camera/CPerspective.h
./src/camera/CPoint.cpp
./src/camera/CPoint.h
./src/camera/vpSubPixel.cpp
./src/camera/vpSubPixel.h
./src/camera/CModelStereo.cpp
./src/camera/CModelStereo.h
./src/camera/CModelStereoXml.cpp
./src/camera/CModelStereoXml.h
./src/camera/CXml.cpp
./src/camera/CXml.h
./src/camera/CPerspectiveXml.cpp
./src/camera/CPerspectiveXml.h
./src/camera/CModelXml.cpp
./src/camera/CModelXml.h
./src/camera/COmniXml.cpp
./src/camera/COmniXml.h
)

ADD_LIBRARY(PhotoVS SHARED ${SRC})
TARGET_LINK_LIBRARIES(PhotoVS ${VISP_LIBRARIES})

#creates the desired image saver and precomputer for VS
add_executable(desired_image_manager src/main_desired_image_manager.cpp src/DesiredImageManager.cpp src/VisualServoTools.cpp)
target_link_libraries(desired_image_manager ${catkin_LIBRARIES} PhotoVS)
add_dependencies(desired_image_manager rosovbc_generate_messages_cpp)

#creates the visual servoing exe
add_executable(visual_servoing src/main_visual_servoing.cpp src/VisualServoing.cpp src/VisualServoTools.cpp)
target_link_libraries(visual_servoing ${catkin_LIBRARIES} PhotoVS)
add_dependencies(visual_servoing rosovbc_generate_messages_cpp)

#creates the visual servoing robust to non-holonomic contraints exe
add_executable(visual_servo_with_rotation src/main_visual_servo_with_rotation.cpp src/VisualServoWithRotation.cpp src/VisualServoWithRotation.cpp  src/VisualServoing.cpp src/VisualServoTools.cpp)
target_link_libraries(visual_servo_with_rotation ${catkin_LIBRARIES} PhotoVS)
add_dependencies(visual_servo_with_rotation rosovbc_generate_messages_cpp)

#add_executable(visual_servoing_with_kinect src/main_visual_servoing_with_kinect.cpp src/VisualServoingWithKinect.cpp src/VisualServoTools.cpp)
#target_link_libraries(visual_servoing_with_kinect ${catkin_LIBRARIES} PhotoVS)
#add_dependencies(visual_servoing_with_kinect rosovbc_generate_messages_cpp)

#add_executable(angle_estimation src/main_angle_estimation.cpp src/VCompassAngleEstimation.cpp src/VisualServoTools.cpp)
#target_link_libraries(angle_estimation ${catkin_LIBRARIES} PhotoVS)
#add_dependencies(angle_estimation rosovbc_generate_messages_cpp)

#add_executable(vs_with_visual_compass src/main_vs_with_visual_compass.cpp src/VCompass.cpp src/VisualServoTools.cpp)
#target_link_libraries(vs_with_visual_compass ${catkin_LIBRARIES} PhotoVS)
#add_dependencies(vs_with_visual_compass rosovbc_generate_messages_cpp)


#add_executable(vs_decoupled src/main_vs_decoupled.cpp src/VCompassDecoupled.cpp src/VisualServoTools.cpp)
#target_link_libraries(vs_decoupled ${catkin_LIBRARIES} PhotoVS)
#add_dependencies(vs_decoupled rosovbc_generate_messages_cpp)

#creates the teleoperation exe
add_executable(teleop_robot src/main_teleop_robot.cpp src/TeleopRobot.cpp)
target_link_libraries(teleop_robot ${catkin_LIBRARIES})
add_dependencies(teleop_robot rosovbc_generate_messages_cpp)


#add_executable(path_manager_enhanced src/main_path_manager_enhanced.cpp src/PathManagerEnhanced.cpp src/VisualServoTools.cpp)
#target_link_libraries(path_manager_enhanced ${catkin_LIBRARIES} PhotoVS)
#add_dependencies(path_manager_enhanced rosovbc_generate_messages_cpp)

#add_executable(path_manager src/main_path_manager.cpp src/PathManager.cpp src/VisualServoTools.cpp)
#target_link_libraries(path_manager ${catkin_LIBRARIES} PhotoVS)
#add_dependencies(path_manager rosovbc_generate_messages_cpp)


#
# adding library
#
#add_library(visual_memory SHARED src/PathManager.cpp src/VisualServoTools.cpp)
#target_link_libraries(visual_memory ${catkin_LIBRARIES} PhotoVS)


#creates the visual path_following_with_rotation exe
add_executable(visual_path_following_with_rotation src/main_visual_path_following_with_rotation.cpp src/VisualPathFollowingWithRotation.cpp src/VisualServoTools.cpp)
target_link_libraries(visual_path_following_with_rotation ${catkin_LIBRARIES} PhotoVS)
add_dependencies(visual_path_following_with_rotation rosovbc_generate_messages_cpp)


#add_executable(visual_path_following src/main_visual_path_following.cpp src/VisualPathFollowing.cpp src/VisualServoTools.cpp)
#target_link_libraries(visual_path_following ${catkin_LIBRARIES} PhotoVS)
#add_dependencies(visual_path_following rosovbc_generate_messages_cpp)


#add_executable(visual_path_following_with_pure_rotation src/main_visual_path_following_with_pure_rotation.cpp src/VisualPathFollowingWithPureRotation.cpp src/VisualServoTools.cpp)
#target_link_libraries(visual_path_following_with_pure_rotation ${catkin_LIBRARIES} PhotoVS)
#add_dependencies(visual_path_following_with_pure_rotation rosovbc_generate_messages_cpp)

#add_executable(visual_path_following_with_kinect src/main_visual_path_following_with_kinect.cpp src/VisualPathFollowingWithKinect.cpp src/VisualServoTools.cpp)
#target_link_libraries(visual_path_following_with_kinect ${catkin_LIBRARIES} PhotoVS Half.so Iex.so IlmImf.so IlmThread.so
# Imath.so jbig.so jpeg.so MCALIB.so MIMGPROC.so MINTERFACE.so
#MIO.so MPOSE.so z.so xml2.so)
#add_dependencies(visual_path_following_with_kinect rosovbc_generate_messages_cpp)

#add_executable(test1 src/main_test.cpp src/test.cpp src/VisualServoTools.cpp)
#target_link_libraries(test1 ${catkin_LIBRARIES} PhotoVS)
#add_dependencies(test1 rosovbc_generate_messages_cpp)

#add_executable(sonar src/main_sonar.cpp src/Sonar.cpp)
#target_link_libraries(sonar ${catkin_LIBRARIES})
#add_dependencies(sonar rosovbc_generate_messages_cpp)

#add_executable(odometry src/main_odometry.cpp src/Odometry.cpp)
#target_link_libraries(odometry ${catkin_LIBRARIES})
#add_dependencies(odometry rosovbc_generate_messages_cpp)

#add_executable(master src/main_master.cpp src/Master.cpp)
#target_link_libraries(master ${catkin_LIBRARIES} PhotoVS)
#add_dependencies(master rosovbc_generate_messages_cpp)

add_executable(visual_path_generation src/main_visual_path_generation.cpp src/visual_path_generation.cpp src/VisualServoTools.cpp)
target_link_libraries(visual_path_generation ${catkin_LIBRARIES} ${VISP_LIBRARIES} PhotoVS)
add_dependencies(visual_path_generation rosovbc_generate_messages_cpp)

include_directories(
  ${catkin_INCLUDE_DIRS}
${CMAKE_CURRENT_SOURCE_DIR}/include/MIXEDVISION
)
