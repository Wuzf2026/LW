cmake_minimum_required(VERSION 3.0.2)
project(lawnwomer)

## Compile as C++11, supported in ROS Noetic
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  RTKData.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  SetMode.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

## Catkin specific configuration
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lawnwomer
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs message_runtime
)

## Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  include/lawnwomer/third_party/gemini335_sdk
  include/lawnwomer/third_party/hesai_jt128_sdk
  include/lawnwomer/third_party/rtk_um982_sdk
)

## Build driver libraries
add_library(gemini335_driver src/gemini335_driver.c)
target_link_libraries(gemini335_driver ${catkin_LIBRARIES} ${CMAKE_CURRENT_SOURCE_DIR}/third_party/gemini335_sdk/libgemini335.so)

add_library(hesai_jt128_driver src/hesai_jt128_driver.c)
target_link_libraries(hesai_jt128_driver ${catkin_LIBRARIES} ${CMAKE_CURRENT_SOURCE_DIR}/third_party/hesai_jt128_sdk/libhesai_jt128.so)

add_library(rtk_um982_driver src/rtk_um982_driver.c)
target_link_libraries(rtk_um982_driver ${catkin_LIBRARIES} ${CMAKE_CURRENT_SOURCE_DIR}/third_party/rtk_um982_sdk/librtk_um982.so)

## Build core node
add_executable(lawnwomer_node src/main_node.cpp)
add_dependencies(lawnwomer_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lawnwomer_node
  ${catkin_LIBRARIES}
  gemini335_driver
  hesai_jt128_driver
  rtk_um982_driver
)

## Install nodes and libraries
install(TARGETS lawnwomer_node gemini335_driver hesai_jt128_driver rtk_um982_driver
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Install Python scripts
catkin_install_python(PROGRAMS scripts/test_serial.py scripts/test_hesai.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install launch and config files
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)