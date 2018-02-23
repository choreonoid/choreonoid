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


set(OPENRTM_DIR ${OPENRTM_DIR} CACHE PATH "set the top directory of OpenRTM-aist")
if(UNIX)
  if(NOT OPENRTM_DIR)
    pkg_check_modules(OPENRTM REQUIRED openrtm-aist)
    message(STATUS "OPENRTM VERSION ${OPENRTM_VERSION}" )
    execute_process(
      COMMAND pkg-config --variable=prefix openrtm-aist
      OUTPUT_VARIABLE OPENRTM_DIR
      RESULT_VARIABLE RESULT
      OUTPUT_STRIP_TRAILING_WHITESPACE)
    message(STATUS "OPENRTM_DIR=${OPENRTM_DIR}")
    if(OPENRTM_VERSION STRLESS "1.2.0")
      set(OPENRTM_DEFINITIONS -DOPENRTM_VERSION11)
    endif()
  endif()

  if(OPENRTM_DIR)
    if(EXISTS "${OPENRTM_DIR}/lib/x86_64-linux-gnu/openrtm-1.1/cmake/OpenRTMConfig.cmake")
      include(${OPENRTM_DIR}/lib/x86_64-linux-gnu/openrtm-1.1/cmake/OpenRTMConfig.cmake)
    elseif(EXISTS "${OPENRTM_DIR}/lib/i386-linux-gnu/openrtm-1.1/cmake/OpenRTMConfig.cmake")
      include(${OPENRTM_DIR}/lib/i386-linux-gnu/openrtm-1.1/cmake/OpenRTMConfig.cmake)
    else()
      include(${OPENRTM_DIR}/lib/openrtm-1.1/cmake/OpenRTMConfig.cmake)
    endif()
    set(OPENRTM_LIBRARY_DIRS ${OPENRTM_DIR}/lib)
    if(OPENRTM_VERSION STRLESS "1.2.0")
      set(OPENRTM_DEFINITIONS -DOPENRTM_VERSION11)
    endif()
  endif()

  set(OPENRTM_PREFIX ${OPENRTM_DIR})

  set(OPENRTM_INCLUDE_DIRS
    ${OPENRTM_DIR}/include 
    ${OPENRTM_DIR}/include/coil-${OPENRTM_VERSION_MAJOR}.${OPENRTM_VERSION_MINOR}
    ${OPENRTM_DIR}/include/openrtm-${OPENRTM_VERSION_MAJOR}.${OPENRTM_VERSION_MINOR}
    ${OPENRTM_DIR}/include/openrtm-${OPENRTM_VERSION_MAJOR}.${OPENRTM_VERSION_MINOR}/rtm/idl
    ${OPENRTM_DIR}/include/openrtm-${OPENRTM_VERSION_MAJOR}.${OPENRTM_VERSION_MINOR}/rtm/ext
    ${OPENRTM_DIR}/rtm # for the Windows installer version
    ${OPENRTM_DIR}/rtm/idl # for the Windows installer version
    ${OPENRTM_DIR}
    )
else()
  set(OPENRTM_PREFIX ${OPENRTM_DIR})
endif()

include_directories(${OPENRTM_INCLUDE_DIRS})


set(IDL_INCLUDE_DIRS 
  ${OPENRTM_PREFIX}/include/openrtm-${OPENRTM_VERSION_MAJOR}.${OPENRTM_VERSION_MINOR}/rtm/idl
  ${OPENRTM_PREFIX}/rtm/idl # for the Windows installer version
  )

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
  make_headers_public(OpenHRPControllerBase.h)

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
    add_definitions(-D__WIN32__ -D__x86__ -D__NT__ -D__OSVERSION__=4 -D_CRT_SECURE_NO_DEPRECATE -D_WIN32_WINNT=0x0500 -DRTC_CORBA_CXXMAPPING11)
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
