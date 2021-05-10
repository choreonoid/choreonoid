if(NOT ENABLE_GUI)
  return()
endif()

option(BUILD_SIMPLE_CONTROLLER_SAMPLES "Building SimpleController samples" ON)
if(NOT BUILD_SIMPLE_CONTROLLER_SAMPLES)
  return()
endif()

set(project_files "")

# SR1
choreonoid_add_simple_controller(SR1MinimumController SR1MinimumController.cpp)
choreonoid_add_simple_controller(SR1WalkPatternController SR1WalkPatternController.cpp)
choreonoid_add_simple_controller(SR1LiftupController SR1LiftupController.cpp)
choreonoid_add_simple_controller(SR1WalkGraspController SR1WalkGraspController.cpp)
set(sr1-pattern-files SR1WalkPattern1.seq SR1WalkPattern2.seq SR1WalkPattern3.seq)
file(COPY ${sr1-pattern-files} DESTINATION ${CNOID_BINARY_SHARE_DIR}/motion/SR1)
install(FILES ${sr1-pattern-files} DESTINATION ${CNOID_SHARE_SUBDIR}/motion/SR1)
list(APPEND project_files
  SR1Minimum.cnoid
  SR1Walk.cnoid
  SR1WalkSlippy.cnoid SlippyMaterialSet.yaml
  SR1Walk2D.cnoid
  SR1Liftup.cnoid
  SR1WalkinHouse.cnoid)

# PA10
choreonoid_add_simple_controller(PA10PickupController PA10PickupController.cpp)
list(APPEND project_files PA10Pickup.cnoid)

# Universal Robots
choreonoid_add_simple_controller(UniversalRobotPickupController UniversalRobotPickupController.cpp)
list(APPEND project_files UR3Pickup.cnoid UR5Pickup.cnoid UR10Pickup.cnoid)

# Crawler
choreonoid_add_simple_controller(SampleCrawlerController SampleCrawlerController.cpp)
choreonoid_add_simple_controller(SampleCrawlerJoystickController SampleCrawlerJoystickController.cpp)
list(APPEND project_files SampleCrawler.cnoid SampleCrawlerJoystick.cnoid SampleCrawlerSlope.cnoid)

# Tank
choreonoid_add_simple_controller(TankJoystickController TankJoystickController.cpp)
list(APPEND project_files Tank.cnoid TankVisionSensors.cnoid)

# DoubleArmV7
choreonoid_add_simple_controller(DoubleArmV7Controller DoubleArmV7Controller.cpp)
list(APPEND project_files DoubleArmV7S.cnoid)

# JACO2
choreonoid_add_simple_controller(Jaco2Controller Jaco2Controller.cpp)

# AizuSpider
choreonoid_add_simple_controller(AizuSpiderController AizuSpiderController.cpp)
list(APPEND project_files AizuSpiderNS.cnoid AizuSpiderSS.cnoid AizuSpiderDS.cnoid)

# AizuWheel
choreonoid_add_simple_controller(AizuWheelController AizuWheelController.cpp)
list(APPEND project_files AizuWheel.cnoid)

# Other samples
choreonoid_add_simple_controller(SpringModelController SpringModelController.cpp)
list(APPEND project_files ControlledSpringModel.cnoid)

choreonoid_add_simple_controller(ConveyorController ConveyorController.cpp)
list(APPEND project_files ConveyorSample.cnoid)

choreonoid_add_simple_controller(CameraSampleController CameraSampleController.cpp)

choreonoid_add_simple_controller(FourWheelCarJoystickController FourWheelCarJoystickController.cpp)
list(APPEND project_files FourWheelCarJoystick.cnoid)

choreonoid_add_simple_controller(LinkPositionController LinkPositionController.cpp)
list(APPEND project_files LinkPositionControl.cnoid)

choreonoid_add_simple_controller(LinkForceController LinkForceController.cpp)
list(APPEND project_files LinkForceControl.cnoid)

choreonoid_add_simple_controller(HandyRockDrillController HandyRockDrillController.cpp)
list(APPEND project_files HandyRockDrill.cnoid)

# Install project files
install(FILES ${project_files} DESTINATION ${CNOID_SHARE_SUBDIR}/project)
