
# @author Shin'ichiro Nakaoka

if(NOT ENABLE_GUI)
  return()
endif()

option(BUILD_AGX_SAMPLES "Building samples of the AgX dynamics" OFF)

if(NOT BUILD_AGX_SAMPLES)
  return()
endif()
if(NOT BUILD_AGX_PLUGIN)
  message(FATAL_ERROR "AgX samples need to build AgXPlugin.")
endif()

# set(CMAKE_BUILD_TYPE Debug)

add_cnoid_simple_controller(AgXTrackedVehicleController AgXTrackedVehicleController.cpp)
configure_file(AgXTrackedVehicle.body ${CNOID_SOURCE_SHARE_DIR}/model/misc COPYONLY)
configure_file(AgXTrackedVehicle.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)
