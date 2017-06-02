option(BUILD_TRACKED_VEHICLE_SAMPLE "Building a tracked vehicle samples" OFF)

if(NOT BUILD_TRACKED_VEHICLE_SAMPLE)
  return()
endif()

add_cnoid_simple_controller(TrackedVehicleController TrackedVehicleController.cpp)
configure_file(TrackedVehicle.body ${CNOID_SOURCE_SHARE_DIR}/model/misc COPYONLY)
configure_file(TrackedVehicle.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)
