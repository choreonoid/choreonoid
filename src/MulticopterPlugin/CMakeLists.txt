if(NOT ENABLE_GUI)
  return()
endif()

option(BUILD_MULTICOPTER_PLUGIN "Building MulticopterPlugin" OFF)
if(NOT BUILD_MULTICOPTER_PLUGIN)
  return()
endif()

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

QT5_WRAP_CPP(qtsources MonitorForm.h)
qt5_wrap_ui(qtforms MonitorForm.ui)

include_directories(${CMAKE_CURRENT_BINARY_DIR})
set(target CnoidMulticopterPlugin)
make_gettext_mofiles(${target} mofiles)
add_cnoid_plugin(${target} SHARED ${headers} ${sources} ${qtsources} ${qtforms} ${mofiles})
target_link_libraries(${target} CnoidBodyPlugin)
apply_common_setting_for_plugin(${target} "${headers}")

if(ENABLE_PYTHON AND USE_PYBIND11)
  add_subdirectory(pybind11)
endif()
