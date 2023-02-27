cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

# Set a default build type if none was specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message("Setting build type to 'RelWithDebInfo' as none was specified.")
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
    "MinSizeRel" "RelWithDebInfo")
endif()

project(test)   


#find required eigen
FIND_PACKAGE( Eigen3 )

#directory of eigen headers
INCLUDE_DIRECTORIES( ${EIGEN3_INCLUDE_DIR} )

find_package(PCL 1.3 REQUIRED COMPONENTS common io visualization filters registration kdtree)     

include_directories(${PCL_INCLUDE_DIRS})                

link_directories(${PCL_LIBRARY_DIRS})                   

add_definitions(${PCL_DEFINITIONS})                    
 
add_executable(test main.cpp)           

target_link_libraries(test ${PCL_LIBRARIES}) 
