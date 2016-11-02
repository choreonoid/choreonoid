if(NOT ENABLE_LUA)
  return()
endif()

add_definitions(-DSOL_SAFE_USERTYPE -DSOL_CHECK_ARGUMENTS)

add_cnoid_lua_module(LuaBase
  LuaBaseModule.cpp
  LuaQtCore.cpp
  LuaQtGui.cpp
  LuaQtEx.cpp
  LuaMainWindow.cpp
  LuaToolBar.cpp
  LuaTimeBar.cpp
  )

target_link_libraries(LuaBase
  CnoidBase
  LuaUtil
  )
