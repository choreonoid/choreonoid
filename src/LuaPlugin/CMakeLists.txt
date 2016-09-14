
# @author Shin'ichiro Nakaoka

option(BUILD_LUA_PLUGIN "Building Lua Plugin" OFF)
if(NOT BUILD_LUA_PLUGIN)
  return()
endif()

pkg_check_modules(LUA lua5.3)
if(NOT LUA_FOUND)
  pkg_check_modules(LUA lua5.2)
endif()

include_directories(${LUA_INCLUDE_DIRS})

#set(CMAKE_BUILD_TYPE Debug)

set(target CnoidLuaPlugin)

set(sources
  LuaPlugin.cpp
  LuaConsoleView.cpp
  )

set(headers
  )

make_gettext_mofiles(${target} mofiles)
add_cnoid_plugin(${target} SHARED ${sources} ${headers} ${mofiles})
target_link_libraries(${target} CnoidBase ${LUA_LIBRARIES})

apply_common_setting_for_plugin(${target} "${headers}")
