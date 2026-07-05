cmake_minimum_required(VERSION 3.8)
project(fdilink_ahrs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

set(SERIAL_INCLUDE_DIR "$ENV{HOME}/.local/include" CACHE PATH "serial include dir")
set(SERIAL_LIBRARY "$ENV{HOME}/.local/lib/libserial.a" CACHE FILEPATH "serial library")

if(NOT EXISTS "${SERIAL_INCLUDE_DIR}/serial/serial.h")
  message(FATAL_ERROR "serial header not found at ${SERIAL_INCLUDE_DIR}/serial/serial.h")
endif()

include_directories(include src ${Eigen3_INCLUDE_DIRS} ${SERIAL_INCLUDE_DIR})

add_library(crc_table STATIC src/crc_table.cpp)
target_include_directories(crc_table PUBLIC include)

add_executable(ahrs_driver src/ahrs_driver.cpp)
target_link_libraries(ahrs_driver ${SERIAL_LIBRARY})
ament_target_dependencies(ahrs_driver
  rclcpp sensor_msgs geometry_msgs nav_msgs std_msgs
  tf2 tf2_geometry_msgs Eigen3
)

add_executable(imu_tf src/imu_tf.cpp)
ament_target_dependencies(imu_tf rclcpp sensor_msgs tf2 tf2_geometry_msgs)

install(TARGETS ahrs_driver imu_tf crc_table
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
ament_package()
