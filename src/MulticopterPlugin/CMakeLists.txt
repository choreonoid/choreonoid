if(NOT ENABLE_GUI)
  return()
endif()

option(BUILD_MULTICOPTER_PLUGIN "Building MulticopterPlugin" OFF)
if(NOT BUILD_MULTICOPTER_PLUGIN)
  return()
endif()

#CMake AUTOMOC
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOMOC_MOC_OPTIONS "-fMulticopterPluginHeader.h")

# header
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")
file(GLOB INC
  "${CMAKE_CURRENT_SOURCE_DIR}/include/*.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp")

# source
file(GLOB SRC
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cxx"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.c")

# form
file(GLOB FORM
  "${CMAKE_CURRENT_SOURCE_DIR}/form/*.ui")

# form -> ui_***.h
qt5_wrap_ui(GEN_FORM ${FORM})
#set(CMAKE_INCLUDE_CURRENT_DIR ON)


set(target CnoidMulticopterPlugin)
make_gettext_mofiles(${target} MO)
add_cnoid_plugin(${target} SHARED ${INC} ${SRC} ${GEN_FORM} ${MO})
target_link_libraries(${target} CnoidBodyPlugin)

apply_common_setting_for_plugin(${target})
