cmake_minimum_required(VERSION 3.16.3)
project(ublox_utils)

find_package(catkin
        REQUIRED COMPONENTS
        roscpp
        nmea_msgs
        ublox_msgs
        rtcm_msgs
        mavros_msgs
        )

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_BUILD_TYPE Release)

############
# Binaries #
############
add_executable(ublox2nmea src/ublox2nmea.cc)
target_link_libraries(ublox2nmea ${catkin_LIBRARIES})

##########
# EXPORT #
##########
install(TARGETS ublox2nmea
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )