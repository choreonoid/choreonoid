# @author Shin'ichiro Nakaoka

option(BUILD_OPENHRP_PLUGIN "Building OpenHRPPlugin" OFF)
option(BUILD_OPENHRP_PLUGIN_FOR_3_0 "Building OpenHRPPlugin for OpenHRP 3.0.x" OFF)

if(NOT BUILD_OPENHRP_PLUGIN)
  return()
elseif(NOT BUILD_OPENRTM_PLUGIN)
  message(FATAL_ERROR "OpenHRP Plugin requires OpenRTMPlugin.")
endif()

# set(CMAKE_BUILD_TYPE Debug)

if(BUILD_OPENHRP_PLUGIN_FOR_3_0)
  set(versions 3.1 3.0)
else()
  set(versions 3.1)
endif()

if(UNIX AND NOT APPLE)
  # Is this necessary to use both plugins for version 3.0 and 3.1 at the same time ?
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-Bsymbolic")
endif()

add_definitions(${OPENRTM_DEFINITIONS})
include_directories(${CMAKE_CURRENT_BINARY_DIR} ${OPENRTM_INCLUDE_DIRS})
link_directories(${OPENRTM_LIBRARY_DIRS})

foreach(version ${versions})

  #-------------------- Sutbs & Skeletons -------------------------
  set(idls
    OpenHRPCommon
    ModelLoader
    DynamicsSimulator
    CollisionDetector
    ViewSimulator
    Controller
    OnlineViewer
    )

  if(version EQUAL 3.0)
    set(defver "OPENHRP_3_0")
  else()
    set(defver "OPENHRP_3_1")
    set(idls ${idls}
      World
      InterpreterService
      ClockGenerator)
  endif()

  set(sources
    OpenHRPControllerBase.cpp
    )
  set(headers
    OpenHRPControllerBase.h
    )

  set(target CnoidOpenHRP${version})

  set(idl_cpp_files "")
  set(idl_h_files "")
  idl_compile_cpp(idl_cpp_files idl_h_files corba/OpenHRP/${version} ${idls})

  add_cnoid_library(${target} STATIC ${sources} ${idl_cpp_files} ${idl_h_files})
  set_target_properties(${target} PROPERTIES COMPILE_DEFINITIONS ${defver})
  target_link_libraries(${target} CnoidCorba)
  apply_common_setting_for_library(${target} "${headers}")

  # Plugin
  set(target CnoidOpenHRP${version}Plugin)

  set(sources
    OpenHRPPlugin.cpp
    DynamicsSimulator_impl.cpp
    OpenHRPControllerItem.cpp
    OnlineViewerServer.cpp
    OpenHRPOnlineViewerItem.cpp
    )

  if(version EQUAL 3.1)
    set(sources ${sources}
      OpenHRPInterpreterServiceItem.cpp
      OpenHRPClockGeneratorItem.cpp
      )
  endif()
  
  set(headers
    exportdecl.h
    )
  
  if(MSVC)
    add_compile_options(${OPENRTM_CFLAGS})
  endif()

  make_gettext_mofiles(${target} mofiles)
  add_cnoid_plugin(${target} SHARED ${sources} ${headers} ${mofiles})
  set_target_properties(${target} PROPERTIES COMPILE_DEFINITIONS ${defver})

  if(version EQUAL 3.0)
    target_link_libraries(${target} CnoidBodyPlugin CnoidOpenHRP${version})
  else()
    target_link_libraries(${target} CnoidOpenRTMPlugin CnoidBodyPlugin CnoidOpenHRP${version})
  endif()

  apply_common_setting_for_plugin(${target} "${headers}")

endforeach()

if(ENABLE_PYTHON)
  if(USE_PYBIND11)
    add_subdirectory(pybind11)
  elseif(USE_BOOST_PYTHON)
    add_subdirectory(boostpython)
  endif()
endif()
