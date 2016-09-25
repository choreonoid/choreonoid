if(NOT ENABLE_LUA)
  return()
endif()

add_definitions(-DSOL_SAFE_USERTYPE -DSOL_CHECK_ARGUMENTS)

add_cnoid_lua_module(LuaBase
  #CLuaBaseModule.cpp
  SolBaseModule.cpp
  #KaguyaBaseModule.cpp
  )

target_link_libraries(LuaBase
  CnoidBase
  ${LUA_LIBRARIES}
  )
