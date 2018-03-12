if(NOT ENABLE_GUI)
  return()
endif()

option(BUILD_MULTICOPTER_PLUGIN "Building MulticopterPlugin" OFF)
if(NOT BUILD_MULTICOPTER_PLUGIN)
  return()
endif()

#CMake AUTOMOC
#set(CMAKE_AUTOMOC ON)
#set(CMAKE_AUTOMOC_MOC_OPTIONS "-fMulticopterPluginHeader.h")

# header
#include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")
#file(GLOB INC
#  "${CMAKE_CURRENT_SOURCE_DIR}/include/*.h"
#  "${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp")

set(headers
  RotorDevice.h
  )

set(sources
  FFCalc_LinkState.cpp
  MulticopterSimulatorItem.cpp
  EventManager.cpp
  FFCalc_calcFluidForce.cpp
  RotorDevice.cpp
  FFCalc_CutoffCoef.cpp
  FluidEnvironment.cpp 
  SimulationManager.cpp
  FFCalc_CutoffCoefImpl.cpp
  LinkManager.cpp
  UtilityImpl.cpp
  FFCalc_FFCalculator.cpp
  MonitorForm.cpp
  FFCalc_GaussQuadratureTriangle.cpp
  MonitorView.cpp
  FFCalc_INormalizedFunction.cpp
  MulticopterPlugin.cpp
  )

# source
#file(GLOB SRC
#  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
#  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cxx"
#  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc"
#  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.c")

# form
#file(GLOB FORM
#  "${CMAKE_CURRENT_SOURCE_DIR}/*.ui")

# form -> ui_***.h
qt5_wrap_ui(qtforms MonitorForm.ui)
#set(CMAKE_INCLUDE_CURRENT_DIR ON)

QTX_WRAP_CPP(qtsources MonitorForm.h)

set(target CnoidMulticopterPlugin)
make_gettext_mofiles(${target} mofiles)
add_cnoid_plugin(${target} SHARED ${headers} ${sources} ${qtsources} ${qtforms} ${mofiles})
target_link_libraries(${target} CnoidBodyPlugin)

apply_common_setting_for_plugin(${target} "${headers}")
