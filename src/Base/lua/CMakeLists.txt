if(NOT ENABLE_LUA)
  return()
endif()

add_cnoid_lua_module(LuaBase
  LuaBaseModule.cpp
  )
target_link_libraries(LuaBase
  CnoidBase
  ${LUA_LIBRARIES}
  )
