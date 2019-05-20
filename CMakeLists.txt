CMAKE_MINIMUM_REQUIRED(VERSION 3.1)
PROJECT(template_prg) # name change it!!

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Debug)# or Release)
ENDIF()

MESSAGE(STATUS "Build type: " ${CMAKE_BUILD_TYPE})
SET(CMAKE_EXPORT_COMPILE_COMMANDS ON )

# c++11 support
add_definitions(-std=c++11)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -pthread")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pthread")
# compile optimization is not turned on by default
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

IF (${CMAKE_BUILD_TYPE} MATCHES "Release")
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g") # no warning
ENDIF()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# specify output directories
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

#---------Some commonly used third-party dependencies------------------------------------#
# opencv
# cmake may find ros opencv
#SET(OpenCV_DIR /usr/local/include/)
FIND_PACKAGE(OpenCV REQUIRED)
MESSAGE(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")

# eigen 
FIND_PACKAGE(Eigen3 3.1 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
# another easy way is:
# include_directories("where eigen is install such as /usr/include/eigen3/ (most common)")

set(THIRD_PARTY_LIBS
        # yaml-cpp
        ${EIGEN3_LIBS}
        ${OpenCV_LIBS}
        ${GLOG_LIBRARY}
        )
# remove certain library
#LIST(APPEND THIRD_PARTY_LIBS ${PCL_LIBRARIES})

include_directories(${PROJECT_SOURCE_DIR}/include)
file(GLOB_RECURSE PRJ_SOURCE_FILE  src/*.cpp)

add_library(${PROJECT_NAME} ${PRJ_SOURCE_FILE} )
target_link_libraries(${PROJECT_NAME} ${THIRD_PARTY_LIBS})

#add_subdirectory(${PROJECT_SOURCE_DIR}/test)
add_executable(pt_time_test test/main.cpp)
target_link_libraries(pt_time_test ${PROJECT_NAME})

# other test: add them!!
# add_executable(other_test test/other_test.cpp)
# target_link_libraries(other_test ${PROJECT_NAME})