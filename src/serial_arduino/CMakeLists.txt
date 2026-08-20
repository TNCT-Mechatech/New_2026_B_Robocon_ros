cmake_minimum_required(VERSION 3.8)
project(serial_arduino)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#----------------------------------------
# Dependencies
#----------------------------------------

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ref_data_msg REQUIRED)
find_package(sensor_data_msg REQUIRED)

#----------------------------------------
# Executable
#----------------------------------------

add_executable(serial_arduino_node
  src/serial_arduino_node.cpp

  ../../SerialBridge/src/source/SerialBridge.cpp
  ../../SerialBridge/src/source/LinuxHardwareSerial.cpp
  ../../SerialBridge/src/source/CobsSerial.cpp
  ../../SerialBridge/src/source/cobs.cpp
  ../../SerialBridge/src/source/Message.cpp
)

#----------------------------------------
# Include directories
#----------------------------------------

target_include_directories(serial_arduino_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../SerialBridge/src
)

#----------------------------------------
# Link libraries
#----------------------------------------

target_link_libraries(serial_arduino_node
    pthread
)

ament_target_dependencies(
    serial_arduino_node
    rclcpp
    sensor_msgs
    ref_data_msg
    sensor_data_msg
)

#----------------------------------------
# Install
#----------------------------------------

install(
    TARGETS serial_arduino_node
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()