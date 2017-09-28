if(NOT ENABLE_GUI)
  return()
endif()

option(BUILD_SIMPLE_CONTROLLER_SAMPLES "Building SimpleController samples" ON)
if(NOT BUILD_SIMPLE_CONTROLLER_SAMPLES)
  return()
endif()

add_cnoid_simple_controller(SR1MinimumController SR1MinimumController.cpp)
configure_file(SR1Minimum.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)

add_cnoid_simple_controller(SR1WalkPatternController SR1WalkPatternController.cpp)
configure_file(SR1Walk.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)
configure_file(SR1WalkPush.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)
configure_file(SR1WalkPush.py ${CNOID_SOURCE_SHARE_DIR}/script COPYONLY)
configure_file(SR1Walk2D.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)

add_cnoid_simple_controller(SR1LiftupController SR1LiftupController.cpp)
configure_file(SR1Liftup.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)
configure_file(../python/AgXsetContactMaterial.py ${CNOID_SOURCE_SHARE_DIR}/script COPYONLY)

add_cnoid_simple_controller(PA10PickupController PA10PickupController.cpp)
configure_file(PA10Pickup.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)

add_cnoid_simple_controller(SampleCrawlerController SampleCrawlerController.cpp)
configure_file(SampleCrawler.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)

add_cnoid_simple_controller(SampleCrawlerJoystickController SampleCrawlerJoystickController.cpp)
configure_file(SampleCrawlerJoystick.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)
configure_file(SampleCrawlerSlope.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)

add_cnoid_simple_controller(TankJoystickController TankJoystickController.cpp)
configure_file(TankJoystick.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)
configure_file(TankJoystickAGX.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)
add_cnoid_simple_controller(LightBlinkController LightBlinkController.cpp)
configure_file(TankJoystickLightBlink.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)

add_cnoid_simple_controller(SR1WalkGraspController SR1WalkGraspController.cpp)
configure_file(SR1WalkinHouse.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)

add_cnoid_simple_controller(ConveyorController ConveyorController.cpp)
configure_file(ConveyorSample.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)

add_cnoid_simple_controller(CameraSampleController CameraSampleController.cpp)

add_cnoid_simple_controller(ThreeWheelerController ThreeWheelerController.cpp)
configure_file(ThreeWheeler.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)

add_cnoid_simple_controller(FourWheelCarJoystickController FourWheelCarJoystickController.cpp)
configure_file(FourWheelCarJoystick.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)
