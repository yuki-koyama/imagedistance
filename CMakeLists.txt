cmake_minimum_required(VERSION 3.1)

project(imagedistance CXX)

set(CMAKE_CXX_STANDARD 11)

######################################################################
# Prerequisites
######################################################################

find_package(Eigen3 REQUIRED)
if((NOT TARGET Eigen3::Eigen) AND (DEFINED EIGEN3_INCLUDE_DIR))
	add_library(AliasEigen3 INTERFACE)
	target_include_directories(AliasEigen3 INTERFACE ${EIGEN3_INCLUDE_DIR})
	add_library(Eigen3::Eigen ALIAS AliasEigen3)
endif()

######################################################################
# Core library
######################################################################

file(GLOB headers ${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp)
file(GLOB sources ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

add_library(imagedistance STATIC ${headers} ${sources})
target_link_libraries(imagedistance PUBLIC Eigen3::Eigen)
target_include_directories(imagedistance PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

install(FILES ${headers} DESTINATION include)
install(TARGETS imagedistance ARCHIVE DESTINATION lib)
