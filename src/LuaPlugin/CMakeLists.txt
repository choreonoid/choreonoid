
# @author Shin'ichiro Nakaoka

option(BUILD_LUA_PLUGIN "Building Lua Plugin" OFF)
if(NOT BUILD_LUA_PLUGIN)
  return()
elseif(NOT ENABLE_LUA)
  message(FATAL_ERROR "LuaPlugin requires ENABLE_LUA")
endif()

#set(CMAKE_BUILD_TYPE Debug)

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
add_cnoid_plugin(${target} SHARED ${sources} ${headers} ${mofiles})
target_link_libraries(${target} CnoidBase ${LUA_LIBRARIES})

apply_common_setting_for_plugin(${target} "${headers}")
