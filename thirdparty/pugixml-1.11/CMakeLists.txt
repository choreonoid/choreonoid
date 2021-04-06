add_library(pugixml STATIC pugixml.cpp)

if(MSVC)
  set_target_properties(pugixml PROPERTIES DEBUG_POSTFIX d)
else()
  set_target_properties(pugixml PROPERTIES COMPILE_OPTIONS -fPIC)
endif()
