cmake_minimum_required(VERSION 2.8)
project(hl_monitoring)

find_package(catkin REQUIRED COMPONENTS
  hl_communication
  )

# Require an external dependency to flycapture library
# FlyCapture is not officially supported on Ubuntu 20.04: need to move to Spinnaker
# - See: https://www.flir.com/support-center/iis/machine-vision/downloads/spinnaker-sdk-flycapture-and-firmware-download/
option(HL_MONITORING_USES_FLYCAPTURE "Use flycapture to build the sources" OFF)

#Enable C++17
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -std=c++17")

# Protobuf generate files with unused parameters
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-parameter")

set(DELEGATE_INCLUDE_DIRS
  include)

set(DELEGATE_LIBRARIES)

if (HL_MONITORING_USES_FLYCAPTURE)
  add_definitions(-DHL_MONITORING_USES_FLYCAPTURE)
  set (DELEGATE_LIBRARIES
    ${DELEGATE_LIBRARIES}
    flycapture
    )
endif(HL_MONITORING_USES_FLYCAPTURE)

catkin_package(
  INCLUDE_DIRS ${DELEGATE_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME} ${DELEGATE_LIBRARIES}
  CATKIN_DEPENDS hl_communication
  )

include_directories(${DELEGATE_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} )

set(SOURCES_DIRECTORIES
  src/hl_monitoring
  src/hl_monitoring/drawers
  )

#Include Sources sub sources
foreach (DIRECTORY ${SOURCES_DIRECTORIES})
  include (${DIRECTORY}/sources.cmake)
  set (PREFIXED_SOURCES)
  foreach (SOURCE ${SOURCES})
    set (PREFIXED_SOURCES ${PREFIXED_SOURCES} ${DIRECTORY}/${SOURCE})
  endforeach (SOURCE)
  set (ALL_SOURCES ${ALL_SOURCES} ${PREFIXED_SOURCES})
endforeach (DIRECTORY)


set(LINKED_LIBRARIES ${catkin_LIBRARIES} ${DELEGATE_LIBRARIES})

add_library (${PROJECT_NAME} SHARED ${PROTO_SOURCES} ${ALL_SOURCES} ${PROTO_DUMMY_FILE})
target_link_libraries(${PROJECT_NAME} ${LINKED_LIBRARIES})


option(BUILD_HL_MONITORING_TOOLS "Building hl_monitoring tools" OFF)

if (BUILD_HL_MONITORING_TOOLS)
  add_executable(intrinsic_calibration tools/intrinsic_calibration.cpp)
  target_link_libraries(intrinsic_calibration ${PROJECT_NAME} ${LINKED_LIBRARIES})

  add_executable(static_calibration tools/static_calibration.cpp)
  target_link_libraries(static_calibration ${PROJECT_NAME} ${LINKED_LIBRARIES})

  add_executable(video_recorder tools/video_recorder.cpp)
  target_link_libraries(video_recorder ${PROJECT_NAME} ${LINKED_LIBRARIES})

  add_executable(basic_monitoring tools/basic_monitoring.cpp)
  target_link_libraries(basic_monitoring ${PROJECT_NAME} ${LINKED_LIBRARIES})

  add_executable(meta_information_tool tools/meta_information_tool.cpp)
  target_link_libraries(meta_information_tool ${PROJECT_NAME} ${LINKED_LIBRARIES})
endif()
