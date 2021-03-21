cmake_minimum_required(VERSION 2.6)
project(cylinder_extract)
find_package(PCL 1.8 REQUIRED)
find_package( Ceres REQUIRED)
include_directories( ${CERES_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
add_executable(cylinder_extract src/main.cpp)

target_link_libraries (cylinder_extract ${PCL_LIBRARIES} ${CERES_LIBRARIES})

#install(TARGETS cylinder_extract RUNTIME DESTINATION bin)
