cmake_minimum_required(VERSION 3.8)
project(can_ankle)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(SERIAL_INCLUDE_DIR "$ENV{HOME}/.local/include" CACHE PATH "serial include dir")
set(SERIAL_LIBRARY "$ENV{HOME}/.local/lib/libserial.a" CACHE FILEPATH "serial library")
set(LIBCONTROLCAN_LIB "${CMAKE_CURRENT_SOURCE_DIR}/lib/libcontrolcan.so" CACHE FILEPATH "controlcan library")

if(NOT EXISTS "${SERIAL_INCLUDE_DIR}/serial/serial.h")
  message(FATAL_ERROR "serial header not found")
endif()
if(NOT EXISTS "${LIBCONTROLCAN_LIB}")
  message(FATAL_ERROR "libcontrolcan.so not found at ${LIBCONTROLCAN_LIB}")
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Torque.msg"
  DEPENDENCIES std_msgs
)

include_directories(include ${SERIAL_INCLUDE_DIR})

macro(add_can_exec name src_file)
  add_executable(${name} ${src_file} ${ARGN})
  target_link_libraries(${name} ${SERIAL_LIBRARY} ${LIBCONTROLCAN_LIB} pthread dl)
  ament_target_dependencies(${name} rclcpp std_msgs geometry_msgs sensor_msgs)
  rosidl_target_interfaces(${name} ${PROJECT_NAME} "rosidl_typesupport_cpp")
endmacro()

macro(add_serial_exec name src_file)
  add_executable(${name} ${src_file})
  target_link_libraries(${name} ${SERIAL_LIBRARY})
  ament_target_dependencies(${name} rclcpp std_msgs geometry_msgs sensor_msgs)
endmacro()

add_can_exec(can_ankleControl_node src/can_ankleControl_node.cpp)
add_can_exec(can_ankleTorqueDriver_node src/can_ankleTorqueDriver_node.cpp)
add_can_exec(can_ankleVelocity_node src/can_ankleVelocity_node.cpp)
add_can_exec(can_test_node src/can_test_node.cpp)
add_serial_exec(serial_ForceSensor_node src/serial_ForceSensor_node.cpp)
add_serial_exec(serial_encoder_node src/serial_encoder_node.cpp)
add_serial_exec(serial_sendCommand_node src/serial_sendCommand_node.cpp)
add_serial_exec(storeTopicMSG_node src/storeTopicMSG_node.cpp)
add_can_exec(storeVelMSG_node src/storeVelMSG_node.cpp)

install(TARGETS
  can_ankleControl_node can_ankleTorqueDriver_node can_ankleVelocity_node
  can_test_node serial_ForceSensor_node serial_encoder_node
  serial_sendCommand_node storeTopicMSG_node storeVelMSG_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY lib/ DESTINATION lib/${PROJECT_NAME})
ament_package()
