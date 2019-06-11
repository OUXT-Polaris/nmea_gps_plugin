cmake_minimum_required(VERSION 2.8.3)
project(nmea_gps_plugin)

find_package(catkin REQUIRED COMPONENTS
  #gazebo
  gazebo_ros
  geodesy
  nmea_msgs
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES nmea_gps_plugin
  CATKIN_DEPENDS gazebo gazebo_ros geodesy nmea_msgs roscpp
#  DEPENDS system_lib
)

find_package(gazebo REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(nmea_gps_plugin src/nmea_gps_plugin)
target_link_libraries(nmea_gps_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

# Install library/executable
install(TARGETS nmea_gps_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)