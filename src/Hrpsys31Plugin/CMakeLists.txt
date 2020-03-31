option(BUILD_HRPSYS31_PLUGIN "Building the Hrpsys31 Plugin" OFF)
if(NOT BUILD_HRPSYS31_PLUGIN)
  return()
else()
  if(NOT BUILD_ROBOT_ACCESS_PLUGIN)
    message(FATAL_ERROR "Hrpsys31Plugin requires RobotAccessPlugin")
  endif()
  if(NOT BUILD_OPENRTM_PLUGIN)
    message(FATAL_ERROR "Hrpsys31Plugin requires OpenRTMPlugin")
  endif()
endif()

set(target CnoidHrpsys31Plugin)

set(sources
  Hrpsys31Plugin.cpp
  Hrpsys31Item.cpp
  )

set(headers
  Hrpsys31Item.h
  )

idl_compile_cpp(idl_cpp_files idl_h_files corba/hrpsys
  RobotHardwareService
  StateHolderService
  LOCAL
  )

include_directories(${CMAKE_CURRENT_BINARY_DIR} ${OPENRTM_INCLUDE_DIRS})
link_directories(${OPENRTM_LIBRARY_DIRS})
make_gettext_mofiles(${target} mofiles)
choreonoid_add_plugin(${target} ${sources} ${idl_cpp_files} ${mofiles} HEADERS ${headers})
target_link_libraries(${target} CnoidRobotAccessPlugin CnoidOpenRTMPlugin)
