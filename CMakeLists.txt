cmake_minimum_required(VERSION 2.8.3)
project(uvc_ros_driver)

add_definitions(-std=c++11 -o3)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(OpenCV 3 REQUIRED COMPONENTS core calib3d highgui)

find_package(libuvc REQUIRED)

# Link against system catkin yaml-cpp if installed.
find_package(PkgConfig)
find_package(yaml_cpp_catkin QUIET)
if(${yaml_cpp_catkin_FOUND})
    message(STATUS "Found yaml_cpp_catkin, using instead of system library.")
    set(YamlCpp_LIBRARIES ${yaml_cpp_catkin_LIBRARIES})
    set(YamlCpp_INCLUDE_DIRS ${yaml_cpp_catkin_INCLUDE_DIRS})
else()
    message(STATUS "No yaml_cpp_catkin, using yaml-cpp system library instead.")
    pkg_check_modules(YamlCpp REQUIRED yaml-cpp>=0.5)
endif()

###########
## Build ##
###########

include_directories(
  include
  ${libuvc_INCLUDE_DIRS}
  /usr/include/libusb-1.0
  src/nodes/c_library
  src/nodes
)

cs_add_library(uvc_ros_driver
  src/uvc_ros_driver.cpp
  src/serial_port.cpp
  src/stereo_homography.cpp
  src/fpga_calibration.cpp
)
target_link_libraries(uvc_ros_driver
  ${YamlCpp_LIBRARIES}
)

cs_add_executable(uvc_ros_driver_node
  src/nodes/uvc_ros_driver_node.cpp
)

target_link_libraries(uvc_ros_driver
  ${libuvc_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_link_libraries(uvc_ros_driver_node
  uvc_ros_driver
)

cs_install()
cs_export()
