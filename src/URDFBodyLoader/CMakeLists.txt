if(NOT ENABLE_URDF)
  return()
endif()

include_directories(${PUGIXML_STATIC_INCLUDE_DIRS})
if(PUGIXML_STATIC_LIBRARY_DIRS)
  link_directories(${PUGIXML_STATIC_LIBRARY_DIRS})
endif()

make_gettext_mofiles(CnoidURDFBodyLoader mofiles)
choreonoid_add_library(CnoidURDFBodyLoader SHARED URDFBodyLoader.cpp HEADERS URDFBodyLoader.h)
target_link_libraries(CnoidURDFBodyLoader PUBLIC CnoidBody PRIVATE ${PUGIXML_STATIC_LIBRARIES})
