if(NOT BUILD_MULTICOPTER_PLUGIN)
  return()
endif()

if(BUILD_SIMPLE_CONTROLLER_SAMPLES)
  add_cnoid_simple_controller(QuadcopterController QuadcopterController.cpp)
  install(FILES QuadcopterJoystick.cnoid DESTINATION ${CNOID_SHARE_SUBDIR}/project)
  target_link_libraries(QuadcopterController CnoidMulticopterPlugin)
endif()
