cmake_minimum_required(VERSION 3.16.3)
project(hl_monitoring)

find_package(OpenCV 4.2 REQUIRED)
# Require an external dependency to flycapture library
# FlyCapture is not officially supported on Ubuntu 20.04: need to move to Spinnaker
# - See: https://www.flir.com/support-center/iis/machine-vision/downloads/spinnaker-sdk-flycapture-and-firmware-download/
option(HL_MONITORING_USES_FLYCAPTURE "Use flycapture to build the sources" OFF)

#Enable C++17
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -std=c++17")

# Protobuf generate files with unused parameters
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-parameter")

set(SOURCES_DIRECTORIES
  src/hl_monitoring
  src/hl_monitoring/drawers
  )

set(ALL_SOURCES
src/hl_monitoring/calibrated_image.cpp
src/hl_monitoring/field.cpp
src/hl_monitoring/top_view_drawer.cpp
src/hl_monitoring/image_provider.cpp
src/hl_monitoring/manual_pose_solver.cpp
src/hl_monitoring/monitoring_manager.cpp
src/hl_monitoring/opencv_image_provider.cpp
src/hl_monitoring/replay_image_provider.cpp
src/hl_monitoring/replay_viewer.cpp
src/hl_monitoring/team_config.cpp
src/hl_monitoring/team_manager.cpp
src/hl_monitoring/drawers/arrow_drawer.cpp
src/hl_monitoring/drawers/captain_drawer.cpp
src/hl_monitoring/drawers/geometry.cpp
src/hl_monitoring/drawers/player_drawer.cpp
src/hl_monitoring/drawers/position_drawer.cpp
src/hl_monitoring/drawers/pose_drawer.cpp
src/hl_monitoring/drawers/team_drawer.cpp
src/hl_monitoring/drawers/text_drawer.cpp
)

if (HL_MONITORING_USES_FLYCAPTURE)
set(SOURCES "${SOURCES}" 
  src/hl_monitoring/flycap_image_provider.cpp
)
endif(HL_MONITORING_USES_FLYCAPTURE)

#Include Sources sub sources
#foreach (DIRECTORY ${SOURCES_DIRECTORIES})
#  include (${DIRECTORY}/sources.cmake)
#  set (PREFIXED_SOURCES)
#  foreach (SOURCE ${SOURCES})
#    set (PREFIXED_SOURCES ${PREFIXED_SOURCES} ${DIRECTORY}/${SOURCE})
#  endforeach (SOURCE)
#  set (ALL_SOURCES ${ALL_SOURCES} ${PREFIXED_SOURCES})
#endforeach (DIRECTORY)


add_library (${PROJECT_NAME} SHARED ${PROTO_SOURCES} ${ALL_SOURCES} ${PROTO_DUMMY_FILE})
target_link_libraries(${PROJECT_NAME} PUBLIC RhIO hl_communication ${OpenCV_LIBS})
target_include_directories(${PROJECT_NAME} PUBLIC
  ${OpenCV_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)

if(HL_MONITORING_USES_FLYCAPTURE)
  target_compile_definitions(${PROJECT_NAME} PUBLIC HL_MONITORING_USES_FLYCAPTURE)
  target_link_libraries(${PROJECT_NAME} PUBLIC flycapture)
endif()

option(BUILD_HL_MONITORING_TOOLS "Building hl_monitoring tools" OFF)

if (BUILD_HL_MONITORING_TOOLS)
  add_executable(intrinsic_calibration tools/intrinsic_calibration.cpp)
  target_link_libraries(intrinsic_calibration ${PROJECT_NAME})

  add_executable(static_calibration tools/static_calibration.cpp)
  target_link_libraries(static_calibration ${PROJECT_NAME})

  add_executable(video_recorder tools/video_recorder.cpp)
  target_link_libraries(video_recorder ${PROJECT_NAME})

  add_executable(basic_monitoring tools/basic_monitoring.cpp)
  target_link_libraries(basic_monitoring ${PROJECT_NAME})

  add_executable(meta_information_tool tools/meta_information_tool.cpp)
  target_link_libraries(meta_information_tool ${PROJECT_NAME})
endif()
