cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
 
project(CVFeatureFinder)

find_package( catkin REQUIRED COMPONENTS roscpp )
FIND_PACKAGE( OpenCV REQUIRED )
find_package( Boost REQUIRED )
FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS})
add_definitions(-std=c++0x)

SET(CVFeatureFinder_SOURCES Controller/main.cpp Model/FeatureFinder.cpp Model/ImageHelper.cpp)
#SET(test_FORMS mainwindow.ui)
SET(CVFeatureFinder_HEADERS Model/FeatureFinder.h Model/ImageHelper.h)

QT4_WRAP_CPP(test_HEADERS_MOC ${test_HEADERS})

include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

ADD_EXECUTABLE(CVFeatureFinder
    ${CVFeatureFinder_SOURCES}
    ${CVFeatureFinder_HEADERS_MOC}
    ${CVFeatureFinder_FORMS_HEADERS}
)


target_link_libraries (CVFeatureFinder ${catkin_LIBRARIES} ${QT_LIBRARIES} ${OpenCV_LIBRARIES} /opt/ros/hydro/lib/libcv_bridge.so)

