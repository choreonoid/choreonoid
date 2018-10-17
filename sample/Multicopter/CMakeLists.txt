if(NOT ENABLE_GUI)
  return()
endif()

option(BUILD_MULTICOPTER_SAMPLES "Building Multicopter samples" OFF)
if(NOT BUILD_MULTICOPTER_SAMPLES)
  return()
endif()

if(NOT BUILD_MULTICOPTER_PLUGIN)
  message(FATAL_ERROR "Multicopter samples need to build MulticopterPlugin.")
endif()

add_cnoid_simple_controller(QuadcopterController QuadcopterController.cpp)
install(FILES QuadcopterJoystick.cnoid DESTINATION ${CNOID_SHARE_SUBDIR}/project)

if(BUILD_OPENRTM_PLUGIN AND BUILD_OPENRTM_SAMPLES)
  add_definitions(${OPENRTM_DEFINITIONS})
  include_directories(${OPENRTM_INCLUDE_DIRS})
  link_directories(${OPENRTM_LIBRARY_DIRS})
  if(MSVC)
    add_compile_options(${OPENRTM_CFLAGS})
    add_definitions(
      -D__WIN32__ -D__x86__ -D__NT__ -D__OSVERSION__=4
      -D_CRT_SECURE_NO_DEPRECATE -D_WIN32_WINNT=0x0500 -DRTC_CORBA_CXXMAPPING11)
  endif()
  add_cnoid_sample_rtc(QuadcopterControllerRTC QuadcopterControllerRTC.cpp)
  add_cnoid_body_io_rtc(QuadcopterIoRTC QuadcopterIoRTC.cpp)
  install(FILES
    OpenRTM-QuadcopterJoystick.cnoid
    OpenRTM-Quadcopter-Terminal.cnoid
    DESTINATION ${CNOID_SHARE_SUBDIR}/project)
endif()
