if(NOT ENABLE_LUA)
  return()
endif()

add_definitions(-DSOL_SAFE_USERTYPE -DSOL_CHECK_ARGUMENTS)

choreonoid_add_lua_module(LuaUtil
  LuaUtil.cpp
  LuaSignal.cpp
  LuaEigenTypes.cpp
  LuaValueTree.cpp
  LuaTask.cpp
  LuaUtilModule.cpp
  HEADERS
  LuaUtil.h
  LuaSignal.h
  LuaEigenTypes.h
  )

target_link_libraries(LuaUtil
  CnoidUtil
  ${LUA_LIBRARIES}
  )
