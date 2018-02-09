cmake_minimum_required(VERSION 2.8.3)
project(ur_kinematics)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
std_msgs roscpp rospy geometry_msgs control_msgs moveit_core moveit_kinematics
  moveit_ros_planning moveit_ros_planning_interface pluginlib tf_conversions message_generation)

find_package(Boost REQUIRED COMPONENTS system)
#############
## Message ##
#############
add_message_files(
  FILES
  vision_data.msg
  vision_data2.msg
)
#############
## Service ##
#############
add_service_files(
  FILES
  service_test.srv
)
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)
################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ur3_kin ur5_kin ur10_kin ur3_moveit_plugin ur5_moveit_plugin ur10_moveit_plugin
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs moveit_core moveit_kinematics moveit_ros_planning moveit_kinematics moveit_ros_planning_interface
    pluginlib tf_conversions message_runtime
  DEPENDS Boost
)

###########
## Build ##
###########

include_directories(SYSTEM ${Boost_INCLUDE_DIR})
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(ur3_kin src/ur_kin.cpp)
set_target_properties(ur3_kin PROPERTIES COMPILE_DEFINITIONS "UR3_PARAMS")

add_library(ur5_kin src/ur_kin.cpp)
set_target_properties(ur5_kin PROPERTIES COMPILE_DEFINITIONS "UR5_PARAMS")

add_library(ur10_kin src/ur_kin.cpp)
set_target_properties(ur10_kin PROPERTIES COMPILE_DEFINITIONS "UR10_PARAMS")


add_library(ur3_moveit_plugin src/ur_moveit_plugin.cpp)
set_target_properties(ur3_moveit_plugin PROPERTIES COMPILE_DEFINITIONS "UR3_PARAMS")
target_link_libraries(ur3_moveit_plugin
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ur3_kin)

add_library(ur5_moveit_plugin src/ur_moveit_plugin.cpp)
set_target_properties(ur5_moveit_plugin PROPERTIES COMPILE_DEFINITIONS "UR5_PARAMS")
target_link_libraries(ur5_moveit_plugin
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ur5_kin)

add_library(ur10_moveit_plugin src/ur_moveit_plugin.cpp)
set_target_properties(ur10_moveit_plugin PROPERTIES COMPILE_DEFINITIONS "UR10_PARAMS")
target_link_libraries(ur10_moveit_plugin
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ur10_kin)
#
add_executable(ur5_pick src/ur5_pick.cpp)
target_link_libraries(ur5_pick ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(ur5_pick PROJECT_NAME)

add_executable(ur5_solver_test src/ur5_solver_test.cpp)
target_link_libraries(ur5_solver_test ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(ur5_solver_test PROJECT_NAME)

add_executable(ur5_trajectory_test src/ur5_trajectory_test.cpp)
target_link_libraries(ur5_trajectory_test ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(ur5_trajectory_test PROJECT_NAME)

add_executable(ur5_PAP_test src/ur5_PAP_test.cpp)
target_link_libraries(ur5_PAP_test ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(ur5_PAP_test PROJECT_NAME)

add_executable(ur5_workspace_test src/ur5_workspace_test.cpp)
target_link_libraries(ur5_workspace_test ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(ur5_workspace_test PROJECT_NAME)

add_executable(ur5_workspace src/ur5_workspace.cpp)
target_link_libraries(ur5_workspace ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(ur5_workspace PROJECT_NAME)

add_executable(ur5_workspace_real src/ur5_workspace_real.cpp)
target_link_libraries(ur5_workspace_real ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(ur5_workspace_real PROJECT_NAME)

add_executable(octomap_loader_test src/octomap_loader_test.cpp)
target_link_libraries(octomap_loader_test ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(octomap_loader_test PROJECT_NAME)

add_executable(ur5_PAP src/ur5_PAP.cpp)
target_link_libraries(ur5_PAP ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(ur5_PAP PROJECT_NAME)

add_executable(ur5_pose_pub src/ur5_pose_pub.cpp)
target_link_libraries(ur5_pose_pub ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(ur5_pose_pub PROJECT_NAME)

add_executable(ur5_pose_sub src/ur5_pose_sub.cpp)
target_link_libraries(ur5_pose_sub ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(ur5_pose_sub PROJECT_NAME)

add_executable(ur5_pose_sub_PAP src/ur5_pose_sub_PAP.cpp)
target_link_libraries(ur5_pose_sub_PAP ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(ur5_pose_sub_PAP PROJECT_NAME)

add_executable(ur5_service_test src/ur5_service_test.cpp)
target_link_libraries(ur5_service_test ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(ur5_service_test PROJECT_NAME)

add_executable(ur5_client_test src/ur5_client_test.cpp)
target_link_libraries(ur5_client_test ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(ur5_client_test PROJECT_NAME)

add_executable(ur5_PAP_service src/ur5_PAP_service.cpp)
target_link_libraries(ur5_PAP_service ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(ur5_PAP_service PROJECT_NAME)

add_executable(ur5_PAP_client src/ur5_PAP_client.cpp)
target_link_libraries(ur5_PAP_client ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(ur5_PAP_client PROJECT_NAME)

#############
## Install ##
#############

install(TARGETS ur3_kin ur5_kin ur10_kin ur3_moveit_plugin ur5_moveit_plugin ur10_moveit_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# install moveit plugin description file
install(FILES ur_moveit_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
