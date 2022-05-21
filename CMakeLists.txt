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
add_executable(04_filter src/04_filter.cpp)
target_link_libraries (04_filter ${PCL_LIBRARIES})
# add_executable(05_index src/05_index.cpp)
# target_link_libraries (05_index ${PCL_LIBRARIES})
add_executable(06_bilateral src/06_bilateral.cpp)
target_link_libraries (06_bilateral ${PCL_LIBRARIES})
add_executable(11_kdtree src/11_kdtree.cpp)
target_link_libraries (11_kdtree ${PCL_LIBRARIES})
add_executable(13_normals src/13_normals.cpp)
target_link_libraries (13_normals ${PCL_LIBRARIES})
add_executable(14_integral_normal src/14_integral_normal.cpp)
target_link_libraries (14_integral_normal ${PCL_LIBRARIES})
add_executable(15_pfh src/15_pfh.cpp)
target_link_libraries (15_pfh ${PCL_LIBRARIES})