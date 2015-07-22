cmake_minimum_required(VERSION 2.8.3)

include_directories(
  ${ROS_LIB_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/libraries
)

# Remove this if using an Arduino without native USB (eg, other than Leonardo)
# add_definitions(-DUSB_CON)

generate_arduino_firmware(joint_controller
  SRCS joint_controller.cpp ${ROS_LIB_DIR}/time.cpp
  BOARD uno
  PORT /dev/ttyACM0
)