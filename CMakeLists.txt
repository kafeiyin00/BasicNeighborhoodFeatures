cmake_minimum_required(VERSION 3.0)

project(BasicNeighborhoodFeatures)
link_directories(D:\\vcpkg\\installed\\x64-windows\\lib)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

find_package(Boost REQUIRED  COMPONENTS system thread filesystem)
include_directories(${Boost_INCLUDE_DIRS})
message("boost include : ${Boost_INCLUDE_DIRS} ")

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
message("eigen include : ${EIGEN3_INCLUDE_DIR} ")

find_package(GLOG REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

find_package(PCL REQUIRED )
include_directories(${PCL_INCLUDE_DIRS})
message("PCL_LIBRARIES: ${PCL_LIBRARIES}")

if (CMAKE_SYSTEM_NAME MATCHES "Linux")
    find_package(Pangolin REQUIRED)
    include_directories(${Pangolin_INCLUDE_DIRS})
    MESSAGE(STATUS "current platform: Linux ")
elseif(CMAKE_SYSTEM_NAME MATCHES "Windows")
endif()

set(DATA_FOLDER_PATH "\"${PROJECT_SOURCE_DIR}/data/\"")
configure_file (
        "${PROJECT_SOURCE_DIR}/Config.h.in"
        "${PROJECT_BINARY_DIR}/Config.h"
)

include_directories(${PROJECT_BINARY_DIR})
include_directories(inc)
include_directories(debugger)

add_library(libbnf
        src/3DFeatureDetection.cpp)

add_subdirectory(test)
add_subdirectory(debugger)
