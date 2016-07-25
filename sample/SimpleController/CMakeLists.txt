
# @author Shin'ichiro Nakaoka

if(NOT ENABLE_GUI)
  return()
endif()

option(BUILD_SIMPLE_CONTROLLER_SAMPLES "Building sample simulation controllers built as simple controllers" ON)

if(NOT BUILD_SIMPLE_CONTROLLER_SAMPLES)
  return()
endif()
if(NOT BUILD_SIMPLE_CONTROLLER_PLUGIN)
  message(FATAL_ERROR "Simple controller samples need to build SimpleControllerPlugin.")
endif()

# set(CMAKE_BUILD_TYPE Debug)

add_cnoid_simple_controller(SR1MinimumController SR1MinimumController.cpp)
configure_file(SR1Minimum.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)

add_cnoid_simple_controller(SR1WalkPatternController SR1WalkPatternController.cpp)
configure_file(SR1Walk.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)
configure_file(SR1WalkPush.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)
configure_file(SR1WalkPush.py ${CNOID_SOURCE_SHARE_DIR}/script COPYONLY)
configure_file(SR1Walk2D.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)

#configure_file(SR1WalkHighGain.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)

add_cnoid_simple_controller(SR1LiftupController SR1LiftupController.cpp)
configure_file(SR1Liftup.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)
configure_file(../python/AgXsetContactMaterial.py ${CNOID_SOURCE_SHARE_DIR}/script COPYONLY)

add_cnoid_simple_controller(PA10PickupController PA10PickupController.cpp)
configure_file(PA10Pickup.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)

add_cnoid_simple_controller(SampleCrawlerController SampleCrawlerController.cpp)
configure_file(SampleCrawler.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)

add_cnoid_simple_controller(SampleCrawlerJoystickController SampleCrawlerJoystickController.cpp)
configure_file(SampleCrawlerJoystick.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)
configure_file(SampleCrawlerSlope_Joystick.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)

add_cnoid_simple_controller(TankJoystickController TankJoystickController.cpp)
configure_file(TankJoystick.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)
add_cnoid_simple_controller(TankLightController TankLightController.cpp)
configure_file(TankJoystickLight.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)

add_cnoid_simple_controller(SR1WalkGraspController SR1WalkGraspController.cpp)
configure_file(SR1WalkinHouse.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)

add_cnoid_simple_controller(ConveyorController ConveyorController.cpp)
configure_file(ConveyorSample.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)

add_cnoid_simple_controller(SampleSVController SampleSV.cpp)

add_cnoid_simple_controller(CameraSampleController CameraSampleController.cpp)

