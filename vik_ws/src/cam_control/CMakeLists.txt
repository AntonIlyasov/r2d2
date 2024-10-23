cmake_minimum_required(VERSION 3.0.2)
project(cam_control)
set(CMAKE_CXX_STANDARD 17)

# Убедитесь, что вы используете Clang
# set(CMAKE_CXX_COMPILER "clang++")
# set(CMAKE_C_COMPILER "clang")

# Добавление флагов для Memory Sanitizer
# set(MSAN_FLAGS "-fsanitize=memory")
# add_compile_options(${MSAN_FLAGS})
# add_link_options(${MSAN_FLAGS})

# # Добавление флагов для Address Sanitizer
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -fno-omit-frame-pointer")
# set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fsanitize=address")
# add_compile_options(-fsanitize=address -fno-omit-frame-pointer)
# add_link_options(-fsanitize=address)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(OpenCV 4.7.0 REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_search_module(GST REQUIRED gstreamer-1.0>=1.4
  gstreamer-sdp-1.0>=1.4
  gstreamer-video-1.0>=1.4
  gstreamer-app-1.0>=1.4)

pkg_check_modules(GST gstreamer-1.0 gstreamer-plugins-base-1.0)

include_directories(${GST_INCLUDE_DIRS})

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
  image_transport
  std_msgs
)


## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

# generate_messages(
#   DEPENDENCIES
#   sensor_msgs#   std_msgs
# )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cam_control
#  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

set(SOURCES src/cam_control.cpp)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GST_INCLUDE_DIRS}
)

add_executable(cam_control src/cam_control_node.cpp ${SOURCES})
target_include_directories (cam_control PRIVATE include/cam_control/ src/)


target_link_libraries(cam_control
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${GST_LIBRARIES}
  rt
)
