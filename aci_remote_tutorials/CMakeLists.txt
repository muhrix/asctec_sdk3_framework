cmake_minimum_required(VERSION 2.8.3)
project(aci_remote_tutorials)

find_package(catkin REQUIRED COMPONENTS
  aci_remote_v100
)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -ansi -std=c99")
set(CMAKE_SHARED_LINKER_FLAGS "-Wl,-O1")

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(aci_tut1_intro src/aci_tut1_intro.c)
add_executable(aci_tut2_recvar src/aci_tut2_recvar.c)
add_executable(aci_tut3_command src/aci_tut3_command.c)
add_executable(aci_tut4_parameter src/aci_tut4_parameter.c)
add_executable(aci_tests_fei src/aci_tests_fei.c)

target_link_libraries(aci_tut1_intro ${catkin_LIBRARIES} -lm -lpthread)
target_link_libraries(aci_tut2_recvar ${catkin_LIBRARIES} -lm -lpthread)
target_link_libraries(aci_tut3_command ${catkin_LIBRARIES} -lm -lpthread)
target_link_libraries(aci_tut4_parameter ${catkin_LIBRARIES} -lm -lpthread)
target_link_libraries(aci_tests_fei ${catkin_LIBRARIES} -lm -lpthread)

install(TARGETS
  aci_tut1_intro aci_tut2_recvar aci_tut3_command aci_tut4_parameter aci_tests_fei
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
