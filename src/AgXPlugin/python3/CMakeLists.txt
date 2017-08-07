
# @author Shizuko Hattori

return()

if(NOT ENABLE_PYTHON3)
  return()
endif()

set(target Py3AgXPlugin)

add_cnoid_python_module(${target}
  PyAgXPlugin.cpp
)

target_link_libraries(${target} CnoidAgXPlugin CnoidPy3Base)
