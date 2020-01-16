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
target_link_libraries(QuadcopterController CnoidMulticopterPlugin)
