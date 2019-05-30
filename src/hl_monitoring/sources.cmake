set(SOURCES
  calibrated_image.cpp
  field.cpp
  top_view_drawer.cpp
  image_provider.cpp
  manual_pose_solver.cpp
  monitoring_manager.cpp
  opencv_image_provider.cpp
  replay_image_provider.cpp
  replay_viewer.cpp
  team_config.cpp
  team_manager.cpp
  utils.cpp
  )

if (HL_MONITORING_USES_FLYCAPTURE)
  set(SOURCES "${SOURCES}" 
    flycap_image_provider.cpp
  )
endif(HL_MONITORING_USES_FLYCAPTURE)
