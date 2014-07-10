cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
 
project(CVFeatureFinder)
set(CMAKE_AUTOMOC ON)

find_package( catkin REQUIRED COMPONENTS roscpp )
FIND_PACKAGE( OpenCV REQUIRED )
find_package( Boost REQUIRED )
FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS})
add_definitions(-std=c++0x)

SET(CVFeatureFinder_SOURCES Controller/main.cpp Controller/CVFF_Controller.cpp Model/FeatureFinder.cpp Model/ImageHelper.cpp View/CVFF_MainWindow.cpp)
SET(CVFeatureFinder_FORMS CVFF_MainWindow.ui)
SET(CVFeatureFinder_HEADERS ${CVFeatureFinder_HDRS} Model/FeatureFinder.h Model/ImageHelper.h Controller/CVFF_Controller.h View/CVFF_MainWindow.h)

QT4_WRAP_CPP(CVFeatureHeader_CPP_FILES ${CVFeatureFinder_MOC_HDRS})
QT4_WRAP_UI(CVFF_FeatureFinder_FORMS_HEADERS ${CVFF_FeatureFinder_FORMS})

include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

ADD_EXECUTABLE(FeatureFinder
    ${CVFeatureFinder_SOURCES}
    ${CVFeatureFinder_HEADERS_MOC}
    ${CVFeatureFinder_FORMS_HEADERS}
)


target_link_libraries (FeatureFinder ${catkin_LIBRARIES} ${QT_LIBRARIES} ${OpenCV_LIBRARIES} /opt/ros/hydro/lib/libcv_bridge.so)

