cmake_minimum_required(VERSION 2.8.3)
project(asctec_hlp_gps)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs asctec_hlp_comm message_filters std_srvs cmake_modules)
find_package(Eigen REQUIRED)
find_package(Boost REQUIRED COMPONENTS signals system thread)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(CMAKE_BUILD_TYPE Release)

catkin_package(
#    DEPENDS Eigen Boost
    CATKIN_DEPENDS roscpp sensor_msgs asctec_hlp_comm message_filters std_srvs 
#    INCLUDE_DIRS
#    LIBRARIES
)

add_executable(hlp_gps_conversion src/gps_conversion.cpp src/gps_conversion_node.cpp)
add_dependencies(hlp_gps_conversion ${catkin_EXPORTED_TARGETS})
target_link_libraries(hlp_gps_conversion ${catkin_LIBRARIES})

add_executable(hlp_set_gps_reference src/set_gps_reference.cpp)
add_dependencies(hlp_set_gps_reference ${catkin_EXPORTED_TARGETS})
target_link_libraries(hlp_set_gps_reference ${catkin_LIBRARIES})


