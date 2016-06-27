# @author Shin'ichiro Nakaoka
if(NOT ENABLE_GUI)
  return()
endif()

option(BUILD_SPRING_MODEL_SAMPLE "Building a spring model sample" OFF)

if(BUILD_SPRING_MODEL_SAMPLE)
  
  configure_file(SpringModel.wrl ${CNOID_SOURCE_SHARE_DIR}/model/misc COPYONLY)
  configure_file(CustomizedSpringModel.wrl ${CNOID_SOURCE_SHARE_DIR}/model/misc COPYONLY)

  if(NOT BUILD_SIMPLE_CONTROLLER_PLUGIN)
    message(WARNING "The spring model sample controller needs SimpleControllerPlugin.")
  else()
    add_cnoid_simple_controller(SpringModelController SpringModelController.cpp)
    configure_file(ControlledSpringModel.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)

    add_cnoid_body_customizer(SpringModelCustomizer SpringModelCustomizer.cpp)
    configure_file(CustomizedSpringModel.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)
  endif()

endif()
