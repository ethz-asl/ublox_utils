cmake_minimum_required(VERSION 3.22)
project(ublox_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nmea_msgs REQUIRED)
find_package(ublox_msgs REQUIRED)
find_package(rtcm_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)

include_directories(include ${ament_cmake_INCLUDE_DIRS})

ament_export_dependencies(rclcpp nmea_msgs ublox_msgs rtcm_msgs mavros_msgs)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_BUILD_TYPE Release)

############
# Binaries #
############
add_executable(ublox2nmea src/ublox2nmea.cc)
ament_target_dependencies(ublox2nmea rclcpp nmea_msgs ublox_msgs rtcm_msgs mavros_msgs)


##########
# Install #
##########
install(TARGETS ublox2nmea
        DESTINATION lib/${PROJECT_NAME})


# install the config directory
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# install the launch directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()