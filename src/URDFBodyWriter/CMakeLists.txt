if(NOT ENABLE_URDF)
  return()
endif()

include_directories(${PUGIXML_STATIC_INCLUDE_DIRS})
if(PUGIXML_STATIC_LIBRARY_DIRS)
  link_directories(${PUGIXML_STATIC_LIBRARY_DIRS})
endif()

choreonoid_add_library(CnoidURDFBodyWriter SHARED URDFBodyWriter.cpp HEADERS URDFBodyWriter.h)
target_link_libraries(CnoidURDFBodyWriter CnoidBody ${PUGIXML_STATIC_LIBRARIES} CnoidAssimpSceneWriter)

choreonoid_add_executable(choreonoid-body2urdf body2urdf.cpp)
target_link_libraries(choreonoid-body2urdf CnoidBody CnoidURDFBodyWriter CnoidAssimpSceneLoader ${Boost_PROGRAM_OPTIONS_LIBRARY})
