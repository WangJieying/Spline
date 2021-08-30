cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_COMPILER "/usr/bin/g++-10")

project(Spline CXX)

# CXX STANDARD
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# include CUDA directories
include_directories(../include)

# include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
# conan_basic_setup()

# create variables to compilable source codes.
set(SOURCE 
  BSplineCurveFitterWindow3.cpp)
  
set(OUTLIB gtapplications gtgraphics gtmathematicsgpu X11 Xext GL EGL png pthread m)

add_library(Spline STATIC ${SOURCE})

  
target_link_libraries(Spline PUBLIC ${OUTLIB})
             