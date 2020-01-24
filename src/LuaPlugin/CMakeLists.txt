# @author Shin'ichiro Nakaoka

if(NOT UNIX)
  return()
endif()

option(BUILD_LUA_PLUGIN "Building Lua Plugin" OFF)
if(NOT BUILD_LUA_PLUGIN)
  return()
elseif(NOT ENABLE_LUA)
  message(FATAL_ERROR "LuaPlugin requires ENABLE_LUA")
endif()

set(target CnoidLuaPlugin)

set(sources
  LuaPlugin.cpp
  LuaInterpreter.cpp
  LuaConsoleView.cpp
  LuaScriptItem.cpp
  )

set(headers
  LuaInterpreter.h
  LuaScriptItem.h
  )

make_gettext_mofiles(${target} mofiles)
choreonoid_add_plugin(${target} ${sources} ${mofiles} HEADERS ${headers})
target_link_libraries(${target} CnoidBase ${LUA_LIBRARIES})
