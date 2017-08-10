# @author Shizuko Hattori

option(BUILD_AGX_PLUGIN "Building AgXPlugin" OFF)

if(NOT BUILD_AGX_PLUGIN)
  return()
endif()

#set(CMAKE_BUILD_TYPE Debug)

IF ("$ENV{AGX_DIR}"  STREQUAL "")
  MESSAGE(WARNING "\n*** The environment initialization file setup_env.bat/bash has not been executed\n*** Errors might occur during runtime!")
ENDIF()

SET(AGX_DIR $ENV{AGX_DIR} CACHE PATH "set the top directory of the AgX Dynamics library")

set(AgX_INCLUDE_DIRS ${AGX_DIR}/include)
set(AgX_LIBRARY_DIRS ${AGX_DIR}/lib)
if(CMAKE_CL_64)
  set(AgX_LIBRARY_DIRS ${AGX_DIR}/lib/x64)
endif()

if(WIN32)
  set(AgX_LIBRARIES debug agxSabred agxCored agxPhysicsd agxModeld) 
else()
  set(AgX_LIBRARIES agxSabre agxCore agxPhysics agxModel) 
endif()

#add_definitions(${AgX_CFLAGS})
include_directories(${AgX_INCLUDE_DIRS} ${AgX_INCLUDE_DIRS}/1.4)
link_directories(${AgX_LIBRARY_DIRS})

set(target CnoidAgXPlugin)

set(sources
  AgXPlugin.cpp
  AgXSimulatorItem.cpp
  )

set(headers
  AgXSimulatorItem.h
  )

make_gettext_mofiles(${target} mofiles)
add_cnoid_plugin(${target} SHARED ${sources} ${headers} ${mofiles})
target_link_libraries(${target} CnoidBodyPlugin ${AgX_LIBRARIES})

get_target_property(compile_flags ${target} COMPILE_FLAGS)
if(NOT compile_flags)
  set(compile_flags "")
endif()
set_target_properties(${target} PROPERTIES COMPILE_FLAGS "${compile_flags} -std=c++11 -msse3")
apply_common_setting_for_plugin(${target} "${headers}")

if(ENABLE_PYTHON)
  if(USE_PYBIND11)
    add_subdirectory(pybind11)
  elseif(USE_BOOST_PYTHON)
    add_subdirectory(boostpython)
  endif()
endif()
