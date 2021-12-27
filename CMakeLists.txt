cmake_minimum_required(VERSION 2.8.3)
project(lsc_planner)

set(CMAKE_CXX_STANDARD 17)
include_directories(${PROJECT_SOURCE_DIR}/include)

#BOOST14
find_package(Boost 1.58 REQUIRED COMPONENTS program_options)

#EIGEN
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

#OCTOMAP
find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_libraries(${OCTOMAP_LIBRARIES})
add_definitions(-DOCTOMAP_NODEBUGOUT)

#PCL
find_package(PCL 1.3 REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#EDT
find_package(dynamicEDT3D REQUIRED)
include_directories(${DYNAMICEDT3D_INCLUDE_DIRS})
link_libraries(${DYNAMICEDT3D_LIBRARIES})

#CPLEX
add_definitions(-DNDEBUG)
add_definitions(-DIL_STD)
set(CPLEX_PREFIX_DIR      /opt/ibm/ILOG/CPLEX_Studio201)
set(CPLEX_INCLUDE_DIR     ${CPLEX_PREFIX_DIR}/cplex/include)
set(CPLEX_LIBRARIES_DIR   ${CPLEX_PREFIX_DIR}/cplex/lib/x86-64_linux/static_pic)
set(CONCERT_INCLUDE_DIR   ${CPLEX_PREFIX_DIR}/concert/include)
set(CONCERT_LIBRARIES_DIR ${CPLEX_PREFIX_DIR}/concert/lib/x86-64_linux/static_pic)
include_directories(${CPLEX_INCLUDE_DIR} ${CONCERT_INCLUDE_DIR})
link_directories(${CPLEX_LIBRARIES_DIR} ${CONCERT_LIBRARIES_DIR})
set(CPLEX_LIBRARIES
  m
  ilocplex
  cplex
  concert
  pthread
  dl
)

#CATKIN
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  rosbag
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  octomap_ros
  octomap_msgs
  dynamic_msgs
  pcl_ros
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp roslib message_runtime octomap_ros octomap_msgs pcl_ros
  DEPENDS OCTOMAP
)

#Message


#RVO2-2D
set(RVO2_2D_SRC
  src/RVO2/Agent.cpp
  src/RVO2/KdTree.cpp
  src/RVO2/Obstacle.cpp
  src/RVO2/RVOSimulator.cpp
)

#RVO2-3D
set(RVO2_3D_SRC
  src/RVO2-3D/Agent.cpp
  src/RVO2-3D/KdTree.cpp
  src/RVO2-3D/RVOSimulator.cpp
)

#Astar-3D
set(Astar_3D_SRC
  src/Astar-3D/astar.cpp
  src/Astar-3D/environmentoptions.cpp
  src/Astar-3D/isearch.cpp
  src/Astar-3D/list.cpp
  src/Astar-3D/map.cpp
  src/Astar-3D/astarplanner.cpp
  src/Astar-3D/node.cpp
)

#openGJK
set(OPENGJK_SRC
  src/openGJK/openGJK.cpp
)

#LSC planner
set(LSC_PLANNER_SRC
  src/param.cpp
  src/mission.cpp
  src/agent_manager.cpp
  src/traj_planner.cpp
  src/traj_optimizer.cpp
  src/grid_based_planner.cpp
  src/collision_constraints.cpp
  src/map_manager.cpp
  ${RVO2_2D_SRC}
  ${RVO2_3D_SRC}
  ${Astar_3D_SRC}
  ${OPENGJK_SRC}
)

#########
# Build #
#########
add_executable(multi_sync_simulator_node
  src/multi_sync_simulator_node.cpp
  src/multi_sync_simulator.cpp
  src/multi_sync_replayer.cpp
  ${LSC_PLANNER_SRC}
)
add_dependencies(multi_sync_simulator_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(multi_sync_simulator_node
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  ${CPLEX_LIBRARIES}
  ${PCL_LIBRARIES}
  stdc++fs
)

add_executable(cmd_publisher_node
  src/cmd_publisher.cpp
  src/cmd_publisher_node.cpp
  src/mission.cpp
)
add_dependencies(cmd_publisher_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(cmd_publisher_node
  ${catkin_LIBRARIES}
)

#add_executable(multi_sync_debugger_node
#  src/multi_sync_debugger_node.cpp
#  src/collision_constraints.cpp
#  ${OPENGJK_SRC}
#)
#add_dependencies(multi_sync_debugger_node ${catkin_EXPORTED_TARGETS})
#target_link_libraries(multi_sync_debugger_node
#  ${catkin_LIBRARIES}
#)