if(NOT ENABLE_LUA OR NOT ENABLE_GUI)
  return()
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
target_link_libraries(${target} ${LUA_LIBRARIES})
