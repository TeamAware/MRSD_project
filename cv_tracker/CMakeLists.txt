cmake_minimum_required(VERSION 2.8.12)
project(cv_tracker)

include(FindPkgConfig)
FIND_PACKAGE(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  message_generation

)
FIND_PACKAGE(CUDA)
FIND_PACKAGE(OpenCV REQUIRED)

pkg_check_modules(HDF5 hdf5)

EXECUTE_PROCESS(
  COMMAND uname -m
  OUTPUT_VARIABLE ARCHITECTURE
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

add_message_files(
  FILES
  image_obj.msg
  image_obj_ranged.msg
  image_obj_tracked.msg
  image_rect.msg
  image_rect_ranged.msg
  obj_label.msg

)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS libdpm_ocv libdpm_ttic scan2image fusion kf runtime_manager calibration_camera_lidar message_runtime std_msgs geometry_msgs
)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

INCLUDE_DIRECTORIES(
  ${catkin_INCLUDE_DIRS}
  lib
)

IF(CUDA_FOUND)
INCLUDE_DIRECTORIES(${CUDA_INCLUDE_DIRS})

IF("${ARCHITECTURE}" MATCHES "^arm")
LINK_DIRECTORIES(/usr/lib/arm-linux-gnueabihf/tegra)
endif()

ELSE()
message("CUDA is not installed. 'cv_tracker' package is not built.")
ENDIF()


#####SSD########
##############################SSD'sFORK of CAFFE NEEDS TO BE PREVIOUSLY COMPILED####################
set(SSD_CAFFE_PATH "/home/teama/catkin_ws/src/ros_caffe/caffe/distribute")
####################################################################################################
if(EXISTS "${SSD_CAFFE_PATH}")

#  INCLUDE_DIRECTORIES(
#    ${SSD_CAFFE_PATH}/include
#    nodes/ssd/include
#  )

IF(HDF5_FOUND)
  INCLUDE_DIRECTORIES(${HDF5_INCLUDE_DIRS})
ENDIF()

  ADD_EXECUTABLE(ssd_unc
    nodes/ssd/ssd_node.cpp
    nodes/ssd/ssd_detector.cpp
  )

  TARGET_LINK_LIBRARIES(ssd_unc
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${CUDA_LIBRARIES}
    ${CUDA_CUBLAS_LIBRARIES}
    ${CUDA_curand_LIBRARY}
    ${SSD_CAFFE_PATH}/lib/libcaffe.so
    glog
  )

  TARGET_INCLUDE_DIRECTORIES( ssd_unc PRIVATE
    ${SSD_CAFFE_PATH}/include
    nodes/ssd/include
  )

  ADD_DEPENDENCIES(ssd_unc
    cv_tracker_generate_messages_cpp
    runtime_manager_generate_messages_cpp
  )
else()
  message("'SSD/Caffe' is not installed. 'ssd_unc' will not be built.")
endif()

## klt_tracker ##
ADD_EXECUTABLE(klt_track
  nodes/klt_track/klt_track.cpp
)

INCLUDE_DIRECTORIES(lib/lktracker)

ADD_LIBRARY(lktracker
  lib/lktracker/LkTracker.cpp
)
TARGET_LINK_LIBRARIES(lktracker
  ${OpenCV_LIBS}
)
ADD_DEPENDENCIES(lktracker
  cv_tracker_generate_messages_cpp
)

TARGET_LINK_LIBRARIES(klt_track
  lktracker
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

ADD_DEPENDENCIES(klt_track
  libdpm_ocv_generate_messages_cpp
)

