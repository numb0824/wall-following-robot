cmake_minimum_required(VERSION 2.8.3)
project(wall_following)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES wall_following
  #CATKIN_DEPENDS roscpp rospy std_msgs
  #DEPENDS system_lib
)


include_directories( include ${catkin_INCLUDE_DIRS}
)




 
 
 add_executable(wallfollowing src/node_wallfollowing.cpp 
                                                                src/wallfollowing.cpp)
 target_link_libraries(wallfollowing ${catkin_LIBRARIES})
 add_dependencies(wallfollowing beginner_tutorials_generate_message_cpp)
