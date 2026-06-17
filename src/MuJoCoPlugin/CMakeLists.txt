option(BUILD_MUJOCO_PLUGIN "Building MuJoCoPlugin" ${ENABLE_MUJOCO})
if(NOT BUILD_MUJOCO_PLUGIN)
  return()
endif()

if(NOT ENABLE_MUJOCO)
  message(FATAL_ERROR "BUILD_MUJOCO_PLUGIN requires ENABLE_MUJOCO to be ON.")
  return()
endif()

set(target CnoidMuJoCoPlugin)

set(sources
  MuJoCoPlugin.cpp
  MuJoCoSimulatorItem.cpp
  )

set(headers
  MuJoCoPlugin.h
  MuJoCoSimulatorItem.h
  exportdecl.h
  )

make_gettext_mofiles(${target} mofiles)
choreonoid_add_plugin(${target} ${sources} ${mofiles} HEADERS ${headers})
target_link_libraries(${target} PUBLIC CnoidBodyPlugin PRIVATE mujoco)

if(ENABLE_PYTHON)
  choreonoid_add_python_binding_subdirectory()
endif()
