if(NOT BUILD_OPENRTM_PLUGIN)
  return()
endif()

if(UNIX)
  pkg_check_modules(OPENRTM REQUIRED openrtm-aist)
  
  execute_process(
    COMMAND pkg-config --variable=prefix openrtm-aist
    OUTPUT_VARIABLE OPENRTM_DIR
    RESULT_VARIABLE RESULT
    OUTPUT_STRIP_TRAILING_WHITESPACE)
  
  execute_process(
    COMMAND pkg-config --variable=rtm_idldir openrtm-aist
    OUTPUT_VARIABLE OPENRTM_IDL_DIRS
    RESULT_VARIABLE RESULT
    OUTPUT_STRIP_TRAILING_WHITESPACE)

  set(OPENRTM_LIBRARY_DIRS ${OPENRTM_DIR}/lib)

elseif(MSVC)
  set(OPENRTM_DIR ${OPENRTM_DIR} CACHE PATH "set the top directory of OpenRTM-aist")
  
  if(NOT OPENRTM_DIR)
    find_package(OpenRTM)
  endif()

  set(OPENRTM_INCLUDE_DIRS
    ${OPENRTM_DIR}/include 
    ${OPENRTM_DIR}/include/coil-${OPENRTM_VERSION_MAJOR}.${OPENRTM_VERSION_MINOR}
    ${OPENRTM_DIR}/include/openrtm-${OPENRTM_VERSION_MAJOR}.${OPENRTM_VERSION_MINOR}
    ${OPENRTM_DIR}/include/openrtm-${OPENRTM_VERSION_MAJOR}.${OPENRTM_VERSION_MINOR}/rtm/idl
    ${OPENRTM_DIR}/include/openrtm-${OPENRTM_VERSION_MAJOR}.${OPENRTM_VERSION_MINOR}/rtm/ext
    ${OPENRTM_DIR}/rtm/idl # for the Windows installer version
    ${OPENRTM_DIR}/rtm # for the Windows installer version
    ${OPENRTM_DIR}
    )
  set(OPENRTM_IDL_DIRS
    ${OPENRTM_DIR}/include/openrtm-${OPENRTM_VERSION_MAJOR}.${OPENRTM_VERSION_MINOR}/rtm/idl
    ${OPENRTM_DIR}/rtm/idl # for the Windows installer version
    )
endif()

if(NOT OPENRTM_DIR)
  message(FATAL_ERROR "Could not find OpenRTM")
endif()

set(OPENRTM_VERSION ${OPENRTM_VERSION} PARENT_SCOPE)
if(OPENRTM_VERSION STRLESS "1.2.0")
  list(APPEND OPENRTM_DEFINITIONS -DOPENRTM_VERSION110)
  list(APPEND OPENRTM_DEFINITIONS -DOPENRTM_VERSION11)
endif()

set(OPENRTM_DEFINITIONS ${OPENRTM_DEFINITIONS} PARENT_SCOPE)
set(OPENRTM_INCLUDE_DIRS ${OPENRTM_INCLUDE_DIRS} PARENT_SCOPE)
set(IDL_INCLUDE_DIRS ${IDL_INCLUDE_DIRS} ${OPENRTM_IDL_DIRS} PARENT_SCOPE)
set(OPENRTM_LIBRARIES ${OPENRTM_LIBRARIES})

add_definitions(${OPENRTM_DEFINITIONS})
include_directories(${OPENRTM_INCLUDE_DIRS})
link_directories(${OPENRTM_LIBRARY_DIRS})

# BodyIoRTC
set(target2 CnoidBodyIoRTC)
set(sources BodyIoRTC.cpp)
set(headers BodyIoRTC.h exportdecl.h)

if(MSVC)
  add_definitions(-DRTC_CORBA_CXXMAPPING11)
endif()

add_cnoid_library(${target2} SHARED ${sources} ${headers})
target_link_libraries(${target2} CnoidBody CnoidCorba ${OPENRTM_LIBRARIES})
apply_common_setting_for_library(${target2} "${headers}")

function(add_cnoid_body_io_rtc)
  set(target ${ARGV0})
  list(REMOVE_AT ARGV 0)

  add_library(${target} SHARED ${ARGV})
  target_link_libraries(${target} CnoidBodyIoRTC)
  
  if(MSVC)
    add_definitions(-DRTC_CORBA_CXXMAPPING11)
  endif()

  set_target_properties(${target} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR}/rtc
    PREFIX "")
  if(ENABLE_INSTALL_RPATH)
    set_target_properties(${target} PROPERTIES INSTALL_RPATH "$ORIGIN/../..")
  endif()

  install(TARGETS ${target}
    RUNTIME DESTINATION ${CNOID_PLUGIN_SUBDIR}/rtc CONFIGURATIONS Release Debug
    LIBRARY DESTINATION ${CNOID_PLUGIN_SUBDIR}/rtc CONFIGURATIONS Release Debug)
endfunction()
