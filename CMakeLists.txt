cmake_minimum_required(VERSION 3.0.0)
project(hrii_vo)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  hrii_utils
  hrii_trajectory_planner
  eigen_conversions
  actionlib
  hrii_gri_interface
  move_base_msgs
  hrii_robot_msgs
  eigen_conversions
)

find_package(matlogger2 REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
 LIBRARIES hrii_dummy_fsm
 CATKIN_DEPENDS
   hrii_utils
   hrii_trajectory_planner
   hrii_gri_interface
   move_base_msgs
   hrii_robot_msgs
   eigen_conversions
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${matlogger2_LIBRARIES}
)


add_executable(${PROJECT_NAME}_vo_test src/three_directional_planar.cpp)
set_target_properties(${PROJECT_NAME}_vo_test PROPERTIES OUTPUT_NAME "three_directional_planar")
add_dependencies(${PROJECT_NAME}_vo_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_vo_test ${catkin_LIBRARIES})




add_executable(${PROJECT_NAME}_three_directional_position_ctrl src/three_directional_position_ctrl.cpp)
set_target_properties(${PROJECT_NAME}_three_directional_position_ctrl PROPERTIES OUTPUT_NAME "three_directional_position_ctrl")
add_dependencies(${PROJECT_NAME}_three_directional_position_ctrl ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_three_directional_position_ctrl ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}only_translation src/only_translation.cpp)
set_target_properties(${PROJECT_NAME}only_translation PROPERTIES OUTPUT_NAME "only_translation")
add_dependencies(${PROJECT_NAME}only_translation ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}only_translation ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}only_rotation src/only_rotation.cpp)
set_target_properties(${PROJECT_NAME}only_rotation PROPERTIES OUTPUT_NAME "only_rotation")
add_dependencies(${PROJECT_NAME}only_rotation ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}only_rotation ${catkin_LIBRARIES})


add_executable(${PROJECT_NAME}_thr_dir_pln_pos_ctr_vertical src/thr_dir_pln_pos_ctr_vertical.cpp)
set_target_properties(${PROJECT_NAME}_thr_dir_pln_pos_ctr_vertical PROPERTIES OUTPUT_NAME "thr_dir_pln_pos_ctr_vertical")
add_dependencies(${PROJECT_NAME}_thr_dir_pln_pos_ctr_vertical ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_thr_dir_pln_pos_ctr_vertical ${catkin_LIBRARIES})

# add_executable(${PROJECT_NAME}circular_position src/circular_position.cpp)
# set_target_properties(${PROJECT_NAME}circular_position PROPERTIES OUTPUT_NAME "circular_position")
# add_dependencies(${PROJECT_NAME}circular_position ${catkin_EXPORTED_TARGETS})
# target_link_libraries(${PROJECT_NAME}circular_position ${catkin_LIBRARIES})


add_executable(${PROJECT_NAME}_square_planar src/square_planar.cpp)
set_target_properties(${PROJECT_NAME}_square_planar PROPERTIES OUTPUT_NAME "square_planar")
add_dependencies(${PROJECT_NAME}_square_planar ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_square_planar ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_square_planar_position_ctrl src/square_planar_position_ctrl.cpp)
set_target_properties(${PROJECT_NAME}_square_planar_position_ctrl PROPERTIES OUTPUT_NAME "square_planar_position_ctrl")
add_dependencies(${PROJECT_NAME}_square_planar_position_ctrl ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_square_planar_position_ctrl ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_logging_node src/logging_node.cpp)
set_target_properties(${PROJECT_NAME}_logging_node PROPERTIES OUTPUT_NAME "logging_node")
add_dependencies(${PROJECT_NAME}_logging_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_logging_node ${catkin_LIBRARIES} matlogger2::matlogger2)


