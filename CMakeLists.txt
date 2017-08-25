cmake_minimum_required(VERSION 2.8.3)
project(hector_manipulator)

if(UNIX)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x -I /usr/include/eigen3")
endif()

find_package(catkin REQUIRED COMPONENTS
  hector_quadrotor_demo
  control_toolbox
  roscpp
  rospy
  std_msgs
  tf
  hector_uav_msgs
  
)


find_package(Boost REQUIRED COMPONENTS system)

find_package(ompl)

catkin_package(
	CATKIN_DEPENDS roscpp rospy tf hector_quadrotor_demo hector_uav_msgs
)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${OMPL_LIBRARIES})


add_executable(quad_manipulator  src/quad_manipulator.cpp src/ControllerCallbacks.cpp)
target_link_libraries(quad_manipulator ${catkin_LIBRARIES})

add_executable(motion_planner_ompl3D src/motion_planner_ompl3D.cpp src/PlannerCallbacks.cpp)
target_link_libraries(motion_planner_ompl3D ${catkin_LIBRARIES} ${OMPL_LIBRARIES})

# In order to use 2D variant, PlannerCallbacks should be adjusted to 2D.
# add_executable(motion_planner_ompl2D src/motion_planner_ompl2D.cpp src/PlannerCallbacks.cpp)
# target_link_libraries(motion_planner_ompl2D ${catkin_LIBRARIES} ${OMPL_LIBRARIES})

add_executable(quad_user_interface src/quad_user_interface.cpp)
target_link_libraries(quad_user_interface ${catkin_LIBRARIES})

add_executable(collision_checker src/collision_checker.cpp)
target_link_libraries(collision_checker ${catkin_LIBRARIES})


