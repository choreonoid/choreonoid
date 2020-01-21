if(NOT ENABLE_GUI)
  return()
endif()

option(BUILD_MULTICOPTER_PLUGIN "Building MulticopterPlugin" OFF)
if(NOT BUILD_MULTICOPTER_PLUGIN)
  return()
endif()

set(headers
  RotorDevice.h
  exportdecl.h
  )

CHOREONOID_MAKE_HEADER_PUBLIC(RotorDevice.h)

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
choreonoid_make_gettext_mo_files(${target} mofiles)
choreonoid_add_plugin(${target} ${sources} ${qtsources} ${qtforms} ${mofiles} HEADERS ${headers})
target_link_libraries(${target} CnoidBodyPlugin)

if(ENABLE_PYTHON AND USE_PYBIND11)
  add_subdirectory(pybind11)
endif()
