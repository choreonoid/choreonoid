if(NOT ENABLE_GUI)
  return()
endif()

option(BUILD_SIMPLE_CONTROLLER_SAMPLES "Building SimpleController samples" ON)
if(NOT BUILD_SIMPLE_CONTROLLER_SAMPLES)
  return()
endif()

set(project_files "")

# SR1 samples
add_cnoid_simple_controller(SR1MinimumController SR1MinimumController.cpp)
add_cnoid_simple_controller(SR1WalkPatternController SR1WalkPatternController.cpp)
add_cnoid_simple_controller(SR1LiftupController SR1LiftupController.cpp)
add_cnoid_simple_controller(SR1WalkGraspController SR1WalkGraspController.cpp)
set(sr1-pattern-files SR1WalkPattern1.seq SR1WalkPattern2.pseq SR1WalkPattern2.seq SR1WalkPattern3.seq)
file(COPY ${sr1-pattern-files} DESTINATION ${CNOID_BINARY_SHARE_DIR}/motion/SR1)
install(FILES ${sr1-pattern-files} DESTINATION ${CNOID_SHARE_SUBDIR}/motion/SR1)
list(APPEND project_files
  SR1Minimum.cnoid SR1Walk.cnoid SR1Walk2D.cnoid SR1Liftup.cnoid SR1WalkinHouse.cnoid)

# PA10 sample
add_cnoid_simple_controller(PA10PickupController PA10PickupController.cpp)
list(APPEND project_files PA10Pickup.cnoid)

# Crawler samples
add_cnoid_simple_controller(SampleCrawlerController SampleCrawlerController.cpp)
add_cnoid_simple_controller(SampleCrawlerJoystickController SampleCrawlerJoystickController.cpp)
list(APPEND project_files SampleCrawler.cnoid SampleCrawlerJoystick.cnoid SampleCrawlerSlope.cnoid)

# Tank samples
add_cnoid_simple_controller(TankJoystickController TankJoystickController.cpp)
list(APPEND project_files Tank.cnoid TankVisionSensors.cnoid AgxTank.cnoid)

# DoubleArmV7 sample
add_cnoid_simple_controller(DoubleArmV7Controller DoubleArmV7Controller.cpp)
list(APPEND project_files DoubleArmV7S.cnoid DoubleArmV7A.cnoid)

# Jaco2 sample
add_cnoid_simple_controller(Jaco2Controller Jaco2Controller.cpp)

# AizuSpider sample
add_cnoid_simple_controller(AizuSpiderController AizuSpiderController.cpp)
list(APPEND project_files
  AizuSpiderNS.cnoid AizuSpiderNA.cnoid AizuSpiderSS.cnoid
  AizuSpiderSA.cnoid AizuSpiderDS.cnoid AizuSpiderDA.cnoid)

# AizuWheel sample
add_cnoid_simple_controller(AizuWheelController AizuWheelController.cpp)
list(APPEND project_files AizuWheel.cnoid)

# Other samples
add_cnoid_simple_controller(ConveyorController ConveyorController.cpp)
list(APPEND project_files ConveyorSample.cnoid)

add_cnoid_simple_controller(CameraSampleController CameraSampleController.cpp)

add_cnoid_simple_controller(FourWheelCarJoystickController FourWheelCarJoystickController.cpp)
list(APPEND project_files FourWheelCarJoystick.cnoid)

add_cnoid_simple_controller(LinkPositionController LinkPositionController.cpp)
list(APPEND project_files LinkPositionControl.cnoid)

add_cnoid_simple_controller(LinkForceController LinkForceController.cpp)
list(APPEND project_files LinkForceControl.cnoid)

add_cnoid_simple_controller(HandyRockDrillController HandyRockDrillController.cpp)
list(APPEND project_files HandyRockDrill.cnoid)

# Install project files
install(FILES ${project_files} DESTINATION ${CNOID_SHARE_SUBDIR}/project)
