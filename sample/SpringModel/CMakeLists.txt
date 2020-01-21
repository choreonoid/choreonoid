if(NOT ENABLE_GUI)
  return()
endif()

option(BUILD_SPRING_MODEL_SAMPLE "Building a spring model sample" OFF)
if(NOT BUILD_SPRING_MODEL_SAMPLE)
  return()
endif()

choreonoid_add_simple_controller(SpringModelController SpringModelController.cpp)
choreonoid_add_body_customizer(SpringModelCustomizer SpringModelCustomizer.cpp)

install(FILES
  ControlledSpringModel.cnoid
  CustomizedSpringModel.cnoid
  DESTINATION ${CNOID_SHARE_SUBDIR}/project)
