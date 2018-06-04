add_cnoid_library(pugixml STATIC src/pugixml.cpp)

# Enable C++11 long long for compilers that are capable of it
if(NOT ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} STRLESS 3.1 AND ";${CMAKE_CXX_COMPILE_FEATURES};" MATCHES ";cxx_long_long_type;")
  target_compile_features(pugixml PUBLIC cxx_long_long_type)
endif()
