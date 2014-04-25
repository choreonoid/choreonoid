# @author Shin'ichiro Nakaoka

option(BUILD_SPRING_MODEL_SAMPLE "Building a spring model sample" OFF)


if(BUILD_SPRING_MODEL_SAMPLE)
  
  if(NOT BUILD_SIMPLE_CONTROLLER_PLUGIN)
    message(FATAL_ERROR "The spring model sample needs SimpleControllerPlugin.")
  endif()

  add_cnoid_simple_controller(SpringModelController SpringModelController.cpp)
  configure_file(ControlledSpringModel.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)
  configure_file(SpringModel.wrl ${CNOID_SOURCE_SHARE_DIR}/model/misc COPYONLY)

  add_cnoid_body_customizer(SpringModelCustomizer SpringModelCustomizer.cpp)
  configure_file(CustomizedSpringModel.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)
  configure_file(CustomizedSpringModel.wrl ${CNOID_SOURCE_SHARE_DIR}/model/misc COPYONLY)

endif()
