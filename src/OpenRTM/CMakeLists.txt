option(BUILD_OPENRTM_PLUGIN "Building OpenRTMPlugin" OFF)
if(NOT BUILD_OPENRTM_PLUGIN)
  return()
elseif(NOT (ENABLE_CORBA AND BUILD_CORBA_PLUGIN))
  message(FATAL_ERROR "OpenRTMPlugin needs to ENABLE_CORBA and BUILD_CORBA_PLUGIN")
endif()

add_definitions(${OPENRTM_DEFINITIONS})
include_directories(${OPENRTM_INCLUDE_DIRS})
link_directories(${OPENRTM_LIBRARY_DIRS})

if(MSVC)
  add_compile_options(${OPENRTM_CFLAGS})
endif()

# The headers and library of basic IDLs
set(target1 CnoidOpenRTM)
idl_compile_cpp(idl_cpp_files idl_h_files corba PointCloud)
add_cnoid_library(${target1} STATIC ${idl_cpp_files} ${idl_h_files})
apply_common_setting_for_library(${target1} "")

# BodyIoRTC
set(target2 CnoidBodyIoRTC)
set(sources BodyIoRTC.cpp)
set(headers BodyIoRTC.h exportdecl.h)

add_cnoid_library(${target2} SHARED ${sources} ${headers})
target_link_libraries(${target2} CnoidBody CnoidCorba CnoidOpenRTM ${OPENRTM_LIBRARIES})
apply_common_setting_for_library(${target2} "${headers}")

function(add_cnoid_body_io_rtc)
  set(target ${ARGV0})
  list(REMOVE_AT ARGV 0)

  add_library(${target} SHARED ${ARGV})
  target_link_libraries(${target} CnoidBodyIoRTC ${OPENRTM_CAMERA_LIBRARY})
  
  if(MSVC)
    add_compile_options(${OPENRTM_CFLAGS})
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

file(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR}/rtc)
