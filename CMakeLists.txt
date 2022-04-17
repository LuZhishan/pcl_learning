cmake_minimum_required(VERSION 3.5 FATAL_ERROR) 

project(pcd_note)
  
find_package(PCL 1.10 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

message(STATUS "*****PCL_INCLUDE_DIRS***** " ${PCL_INCLUDE_DIRS}) # 查看PCL版本是否正确
message(STATUS "*****PCL_LIBRARY_DIRS***** " ${PCL_LIBRARY_DIRS})
message(STATUS "*****PCL_DEFINITIONS ***** " ${PCL_DEFINITIONS})


add_executable(01_io src/01_io.cpp)
target_link_libraries (01_io ${PCL_LIBRARIES})
add_executable(02_viewer src/02_viewer.cpp)
target_link_libraries (02_viewer ${PCL_LIBRARIES})
add_executable(03_matrix src/03_matrix.cpp)
target_link_libraries (03_matrix ${PCL_LIBRARIES})