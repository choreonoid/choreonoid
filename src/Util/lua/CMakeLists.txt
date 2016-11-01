if(NOT ENABLE_LUA)
  return()
endif()

add_definitions(-DSOL_SAFE_USERTYPE -DSOL_CHECK_ARGUMENTS)

add_cnoid_lua_module(LuaUtil
  LuaUtil.cpp
  LuaSignal.cpp
  LuaEigenTypes.cpp
  LuaValueTree.cpp
  LuaTask.cpp
  LuaUtilModule.cpp
  )

target_link_libraries(LuaUtil
  CnoidUtil
  ${LUA_LIBRARIES}
  )

set(headers
  LuaUtil.h
  LuaSignal.h
  LuaEigenTypes.h
  )

apply_common_setting_for_lua_module(LuaUtil "${headers}")
