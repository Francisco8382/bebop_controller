cmake_minimum_required(VERSION 3.0.2)
project(bebop_controller)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  gazebo_msgs
  mav_msgs
  nav_msgs
  std_msgs
  roscpp
  sensor_msgs
  cmake_modules
  message_generation
  rospy
)

find_package(
  Eigen REQUIRED
)

generate_messages(
  DEPENDENCIES
  std_msgs
  mav_msgs
)

catkin_package(
  INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
  LIBRARIES base_controller
  CATKIN_DEPENDS geometry_msgs mav_msgs nav_msgs roscpp sensor_msgs std_msgs message_runtime
  DEPENDS Eigen 
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS scripts/gazebo.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/plot.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_library(base_controller
   src/library/base_controller.cpp
)

add_library(waypoint
   src/library/waypoint.cpp
)

target_link_libraries(base_controller ${catkin_LIBRARIES})
add_dependencies(base_controller ${catkin_EXPORTED_TARGETS})

target_link_libraries(waypoint ${catkin_LIBRARIES})
add_dependencies(waypoint ${catkin_EXPORTED_TARGETS})

add_executable(data_to_csv src/nodes/data_to_csv.cpp)
add_dependencies(data_to_csv ${catkin_EXPORTED_TARGETS})
target_link_libraries(data_to_csv ${catkin_LIBRARIES})

add_executable(proportional_controller src/nodes/proportional_controller.cpp)
add_dependencies(proportional_controller ${catkin_EXPORTED_TARGETS})
target_link_libraries(proportional_controller
  base_controller ${catkin_LIBRARIES})

add_executable(square_root_controller src/nodes/square_root_controller.cpp)
add_dependencies(square_root_controller ${catkin_EXPORTED_TARGETS})
target_link_libraries(square_root_controller
  base_controller ${catkin_LIBRARIES})

add_executable(pid_controller_twist src/nodes/pid_controller_twist.cpp)
add_dependencies(pid_controller_twist ${catkin_EXPORTED_TARGETS})
target_link_libraries(pid_controller_twist
  base_controller ${catkin_LIBRARIES})

add_executable(pid_controller_angles src/nodes/pid_controller_angles.cpp)
add_dependencies(pid_controller_angles ${catkin_EXPORTED_TARGETS})
target_link_libraries(pid_controller_angles
  base_controller ${catkin_LIBRARIES})

add_executable(citc_controller_twist src/nodes/citc_controller_twist.cpp)
add_dependencies(citc_controller_twist ${catkin_EXPORTED_TARGETS})
target_link_libraries(citc_controller_twist
  base_controller ${catkin_LIBRARIES})

add_executable(citc_controller_angles src/nodes/citc_controller_angles.cpp)
add_dependencies(citc_controller_angles ${catkin_EXPORTED_TARGETS})
target_link_libraries(citc_controller_angles
  base_controller ${catkin_LIBRARIES})

add_executable(sinusoidal src/nodes/sinusoidal.cpp)
add_dependencies(sinusoidal ${catkin_EXPORTED_TARGETS})
target_link_libraries(sinusoidal
  waypoint ${catkin_LIBRARIES})

install(TARGETS base_controller waypoint
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS proportional_controller square_root_controller 
  pid_controller_twist pid_controller_angles citc_controller_twist 
  citc_controller_angles data_to_csv sinusoidal
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DIRECTORY include/bebop_simulator_msgs/
  DIRECTORY include/bebop_msgs/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
