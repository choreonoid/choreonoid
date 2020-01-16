if(NOT BUILD_MULTICOPTER_PLUGIN)
  return()
endif()

if(BUILD_SIMPLE_CONTROLLER_SAMPLES)
  add_cnoid_simple_controller(QuadcopterController QuadcopterController.cpp)
  install(FILES QuadcopterJoystick.cnoid DESTINATION ${CNOID_SHARE_SUBDIR}/project)
  target_link_libraries(QuadcopterController CnoidMulticopterPlugin)
endif()

if(BUILD_OPENRTM_PLUGIN AND BUILD_OPENRTM_SAMPLES)
  add_definitions(${OPENRTM_DEFINITIONS})
  include_directories(${OPENRTM_INCLUDE_DIRS})
  link_directories(${OPENRTM_LIBRARY_DIRS})
  if(MSVC)
    add_compile_options(${OPENRTM_CFLAGS})
  endif()
  add_cnoid_sample_rtc(QuadcopterControllerRTC QuadcopterControllerRTC.cpp)
  add_cnoid_body_io_rtc(QuadcopterIoRTC QuadcopterIoRTC.cpp)
  target_link_libraries(QuadcopterIoRTC CnoidMulticopterPlugin)
  install(FILES
    OpenRTM-QuadcopterJoystick.cnoid
    OpenRTM-Quadcopter-Terminal.cnoid
    QuadcopterSystem.xml
	Quadcopter-RTSystem.xml
    DESTINATION ${CNOID_SHARE_SUBDIR}/project)
endif()
