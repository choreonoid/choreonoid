if(WIN32)
  option(BUILD_PYTHON3_PLUGIN "Building Python3Plugin" OFF)
else()
  option(BUILD_PYTHON3_PLUGIN "Building Python3Plugin" ON)
endif()

if(NOT BUILD_PYTHON3_PLUGIN)
  return()
elesif(NOT ENABLE_PYTHON3)
  message(FATAL_ERROR "Python3Plugin needs to ENABLE_PYTHON3")
endif()

set(target CnoidPython3Plugin)

set(sources
  PythonPlugin.cpp
  Python3Executor.cpp
  Python3ScriptItem.cpp
  Python3ScriptItemImpl.cpp
  Python3ConsoleView.cpp
  )

set(headers
  Python3Plugin.h
  Python3Executor.h
  Python3ScriptItem.h
  Python3ScriptItemImpl.h
  )

make_gettext_mofiles(${target} mofiles)
add_cnoid_plugin(${target} SHARED ${sources} ${headers} ${mofiles})
target_link_libraries(${target} CnoidBase ${PYTHON3_LIBRARIES})

apply_common_setting_for_plugin(${target} "${headers}")

configure_file(rbimporter.py ${PROJECT_BINARY_DIR}/${CNOID_PYTHON3_SUBDIR}/cnoid/ COPYONLY)
install(FILES rbimporter.py DESTINATION ${CNOID_PYTHON3_SUBDIR}/cnoid
  CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel)

add_subdirectory(python3)
