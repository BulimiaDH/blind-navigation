CMAKE_MINIMUM_REQUIRED(VERSION 3.0)

ADD_DEFINITIONS(-std=c++11)
ADD_DEFINITIONS(-O2 -w)

PROJECT(TRACKMAP)

SET(SOURCE_CODE_DIR ../../cmake_src)
SET(VO_PART ${SOURCE_CODE_DIR}/VO)

SET(SRC trackGroundTest.cpp trackMap.cpp rotationEstimation.cpp ${VO_PART}/View.cpp ${VO_PART}/Feature.cpp ${SOURCE_CODE_DIR}/CNNExtractor.cpp ${SOURCE_CODE_DIR}/imageMatcher.cpp ${SOURCE_CODE_DIR}/PS4CameraManager.cpp ${SOURCE_CODE_DIR}/ps4eye.cpp ${SOURCE_CODE_DIR}/PS4Client.cpp  ${SOURCE_CODE_DIR}/SerialDevice.cpp ${SOURCE_CODE_DIR}/tcpconnector.cpp  ${SOURCE_CODE_DIR}/tcpstream.cpp ${SOURCE_CODE_DIR}/staticMap.cpp)

SET(INCLUDE_DIR  /home/blindfind/gitlab/caffe/build/install/include /usr/local/cuda/include /home/blindfind/Documents/Blindfind3/include/VO /usr/include/eigen3 /usr/local/include/isam /home/blindfind/Documents/Blindfind3/include )

find_package(OpenCV REQUIRED)
find_package(DBoW3 REQUIRED)

SET(INCLUDE_DIR ${INCLUDE_DIR} ${OPENCV_INCLUDE_DIRS})
SET(SHARED_LIBRARY   m pthread glog boost_system boost_filesystem caffe  isam cholmod ${OpenCV_LIBS} ${DBoW3_LIBS} usb-custom udev)
SET(LIB_DIR festival/src/lib speech_tools/lib  ${DBoW3_LIB_DIR} ../../3rdParty /usr/lib/x86_64-linux-gnu /home/blindfind/gitlab/caffe/build/install/lib)

INCLUDE_DIRECTORIES(${INCLUDE_DIR})
LINK_DIRECTORIES(${LIB_DIR})

ADD_EXECUTABLE(trackMapTest ${SRC})

TARGET_LINK_LIBRARIES(trackMapTest ${SHARED_LIBRARY})
