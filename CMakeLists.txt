cmake_minimum_required(VERSION 2.8)
set(SOLUTIONTITLE Multimodal CACHE TYPE STRING)
project(${SOLUTIONTITLE})
file(GLOB FILES_SRC
    "src/*.cpp"
    "src/*.h"
)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 /bigobj /EHa")
set(VVRFRAMEWORK_DIR "" CACHE PATH "Location of VVR Framework")
find_package( OpenCV REQUIRED )

include_directories(${CMAKE_SOURCE_DIR})
include_directories(${VVRFRAMEWORK_DIR}/include)
include_directories(${VVRFRAMEWORK_DIR}/include/VVRScene)
include_directories(${VVRFRAMEWORK_DIR}/include/GeoLib)
include_directories(${VVRFRAMEWORK_DIR}/include/MathGeoLib)
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(${SOLUTIONTITLE} ${FILES_SRC})
target_link_libraries(
${SOLUTIONTITLE} 
${VVRFRAMEWORK_DIR}/lib/VVRScene_d.lib 
${VVRFRAMEWORK_DIR}/lib/GeoLib_d.lib 
${VVRFRAMEWORK_DIR}/lib/MathGeoLib_d.lib
)
target_link_libraries( ${SOLUTIONTITLE} ${OpenCV_LIBS} )
 
add_custom_command(TARGET ${SOLUTIONTITLE} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory "${VVRFRAMEWORK_DIR}/lib" ${CMAKE_BINARY_DIR}/$<CONFIG>)
add_custom_command(TARGET ${SOLUTIONTITLE} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory "${OpenCV_DIR}/bin/Debug" ${CMAKE_BINARY_DIR}/$<CONFIG>)

 