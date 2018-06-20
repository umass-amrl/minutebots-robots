cmake_minimum_required(VERSION 2.8.11)

PROJECT(minutebots-robots)

# Flag for toggling test compilation on or off.
OPTION(test "Build all tests." ON)

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
SET(CMAKE_INCLUDE_CURRENT_DIR ON)

INCLUDE(FindProtobuf)

FIND_PACKAGE(Protobuf REQUIRED)


SET(CMAKE_CXX_FLAGS "-std=c++11 -Wall -Wsign-compare -Werror -g")
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -rdynamic")
SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -rdynamic")

IF(${CMAKE_BUILD_TYPE} MATCHES "Release")
  MESSAGE(STATUS "Additional Flags for Release mode")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -openmp -fno-builtin-malloc")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fno-builtin-calloc")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fno-builtin-realloc")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fno-builtin-free -O3 -DNDEBUG")
  #SET(BUILD_SPECIFIC_LIBRARIES "tcmalloc")
ELSEIF(${CMAKE_BUILD_TYPE} MATCHES "Debug")
  MESSAGE(STATUS "Additional Flags for Debug mode")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g -O1")
  SET(BUILD_SPECIFIC_LIBRARIES "")
ENDIF()

SET(libs
    ${PROTOBUF_LIBRARY} glog rt pthread ${BUILD_SPECIFIC_LIBRARIES})

FILE(GLOB PROTO_FILES src/proto/*.proto)

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})

INCLUDE_DIRECTORIES(src/)
INCLUDE_DIRECTORIES(${PROTOBUF_INCLUDE_DIR})

ADD_LIBRARY(protobuf_library ${PROTO_SRCS} ${PROTO_PY})
TARGET_LINK_LIBRARIES(protobuf_library ${libs})

ADD_EXECUTABLE(direct_drive_tester
    src/radio/direct_drive_tester.cc
    src/joystick/joystick.cc
    src/serial/serial.cc
    src/util/timer.cc
    src/radio/crc.cc
)
TARGET_LINK_LIBRARIES(direct_drive_tester ${libs})
