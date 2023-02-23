cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(test)   
                            
find_package(PCL 1.3 REQUIRED COMPONENTS common io visualization filters)     

include_directories(${PCL_INCLUDE_DIRS})                

link_directories(${PCL_LIBRARY_DIRS})                   

add_definitions(${PCL_DEFINITIONS})                    
 
add_executable(test main.cpp)           

target_link_libraries(test ${PCL_LIBRARIES}) 
