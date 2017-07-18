cmake_minimum_required(VERSION 2.8.3)
project(hector_manipulator)

if(UNIX)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x")
endif()

find_package(catkin REQUIRED COMPONENTS
  hector_quadrotor_demo
  roscpp
  std_msgs
  tf
  hector_uav_msgs
)


find_package(Boost REQUIRED COMPONENTS system)


catkin_package(
	CATKIN_DEPENDS roscpp tf hector_quadrotor_demo hector_uav_msgs
)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

#add_executable(pose_calculator src/pose_calculator.cpp)
#target_link_libraries(pose_calculator ${catkin_LIBRARIES})

#add_executable(uav_master src/uav_master.cpp)
#target_link_libraries(uav_master ${catkin_LIBRARIES})

#add_executable(vel_calculator src/vel_calculator.cpp)
#target_link_libraries(vel_calculator ${catkin_LIBRARIES})

add_executable(set_goal src/set_goal.cpp)
target_link_libraries(set_goal ${catkin_LIBRARIES})
add_executable(basic_flyer src/basic_flyer.cpp)
target_link_libraries(basic_flyer ${catkin_LIBRARIES})
