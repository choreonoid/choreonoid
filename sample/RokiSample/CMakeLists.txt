
# @author Shin'ichiro Nakaoka

if(NOT BUILD_ROKI_PLUGIN)
  return()
endif()

add_cnoid_simple_controller(Arm2dofController Arm2dofController.cpp)
configure_file(RokiArm2Dof.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)
