cmake_minimum_required(VERSION 2.8.3)
project(hand_eye_calibration)

## Use C++11
add_definitions(--std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    pcl_ros
    sensor_msgs
    geometry_msgs
    cv_bridge
)

find_package( OpenCV REQUIRED )

find_package(Eigen3)
if(NOT EIGEN3_FOUND)
    # Fallback to cmake_modules
    find_package(Eigen REQUIRED)
    set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
    set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
    # ...
endif()

#LINK_DIRECTORIES($ENV{HALCONROOT}/lib/$ENV{HALCONARCH})

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake matching_config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    INCLUDE_DIRS
    include
#  LIBRARIES
    CATKIN_DEPENDS
    pcl_ros
    roscpp
    sensor_msgs
    geometry_msgs
    cv_bridge
#    asr_halcon_bridge

#  DEPENDS
    DEPENDS
    EIGEN3
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
#    $ENV{HALCONROOT}/include
#    /opt/halcon/include
#    /opt/halcon/include/halconcpp
    ${EIGEN3_INCLUDE_DIRS}
)

file(GLOB SOURCES "src/hand_eye_calibration/*.cpp")

## Declare a C++ library
add_library(${PROJECT_NAME}
            ${SOURCES})

## Declare cpp executables
add_executable(
        ${PROJECT_NAME}_test
        src/reference_test.cpp
)

add_executable(
        ${PROJECT_NAME}_calibrate
        src/calibrate.cpp
)

add_executable(
        recorder
        src/record_event.cpp
)

## Specify libraries to link executable targets against
target_link_libraries(
    ${PROJECT_NAME}_test
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

target_link_libraries(
    ${PROJECT_NAME}_calibrate
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

target_link_libraries(
    recorder
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)


