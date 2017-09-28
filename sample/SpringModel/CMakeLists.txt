# @author Shin'ichiro Nakaoka
if(NOT ENABLE_GUI)
  return()
endif()

option(BUILD_SPRING_MODEL_SAMPLE "Building a spring model sample" OFF)

if(BUILD_SPRING_MODEL_SAMPLE)
  
  configure_file(SpringModel.body ${CNOID_SOURCE_SHARE_DIR}/model/misc COPYONLY)
  configure_file(CustomizedSpringModel.body ${CNOID_SOURCE_SHARE_DIR}/model/misc COPYONLY)

  add_cnoid_simple_controller(SpringModelController SpringModelController.cpp)
  configure_file(ControlledSpringModel.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)

  add_cnoid_body_customizer(SpringModelCustomizer SpringModelCustomizer.cpp)
  configure_file(CustomizedSpringModel.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)

endif()
