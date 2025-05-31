set(sources
  SampleCameraEffectPlugin.cpp
  SampleCameraEffectSimulatorItem.cpp
)

set(headers
  SampleCameraEffectSimulatorItem.h
)

set(target CnoidSampleCameraEffectPlugin)
choreonoid_make_gettext_mo_files(${target} mofiles)
choreonoid_add_plugin(${target} ${sources} ${mofiles} HEADERS ${headers})
target_link_libraries(${target} PUBLIC CnoidGLVisionSimulatorPlugin)