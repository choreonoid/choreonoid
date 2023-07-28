option(BUILD_HRP4C_HANDLER "Building HRP4CHandler" OFF)

if(BUILD_HRP4C_HANDLER)
  choreonoid_add_body_handler(HRP4CHandler HRP4CHandler.cpp)
endif()
