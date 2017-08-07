
# @author Shizuko Hattori

if(NOT ENABLE_PYTHON)
  return()
endif()

set(target PyAgXPlugin)

add_cnoid_python_module(${target}
  PyAgXPlugin.cpp
)

target_link_libraries(${target} CnoidAgXPlugin CnoidPyBase)
