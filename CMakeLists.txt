cmake_minimum_required (VERSION 3.5)
project(astar)

#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 -pthread -fno-elide-constructors")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 -pthread")
#SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -pg")
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg -std=c++11 -O3 -pthread -Wall")
#SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pg")
#SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -pg")

find_package(OpenCV REQUIRED)
include_directories(
  ${OpenCV_INCLUDE_DIRS})

aux_source_directory(./src ALL_SOURCES)
add_executable(${PROJECT_NAME} ${ALL_SOURCES})
#target_link_libraries(${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS} ${catkin_LIBRARIES})
