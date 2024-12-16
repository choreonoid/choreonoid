set(target PyGLVisionSimulatorPlugin)

choreonoid_add_python_module(${target}
  PyGLVisionSimulatorItem.cpp
)

target_link_libraries(${target} CnoidGLVisionSimulatorPlugin CnoidPyBase)
