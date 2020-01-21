if(NOT ENABLE_LUA)
  return()
endif()

add_definitions(-DSOL_SAFE_USERTYPE -DSOL_CHECK_ARGUMENTS -ftemplate-depth=1300)

choreonoid_add_lua_module(LuaBase
  LuaBaseModule.cpp
  LuaQtCore.cpp
  LuaQtGui.cpp
  LuaQtEx.cpp
  LuaMainWindow.cpp
  LuaToolBar.cpp
  LuaTimeBar.cpp
  LuaItems.cpp
  LuaViews.cpp
  LuaItemTreeView.cpp
  )

target_link_libraries(LuaBase
  CnoidBase
  LuaUtil
  )
