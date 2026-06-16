if(NOT ENABLE_SDF)
  return()
endif()

include_directories(${PUGIXML_STATIC_INCLUDE_DIRS})
if(PUGIXML_STATIC_LIBRARY_DIRS)
  link_directories(${PUGIXML_STATIC_LIBRARY_DIRS})
endif()

make_gettext_mofiles(CnoidSDFBodyLoader mofiles)
choreonoid_add_library(CnoidSDFBodyLoader SHARED SDFBodyLoader.cpp HEADERS SDFBodyLoader.h exportdecl.h)
target_link_libraries(CnoidSDFBodyLoader PUBLIC CnoidBody PRIVATE ${PUGIXML_STATIC_LIBRARIES})
