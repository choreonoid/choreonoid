if(NOT ENABLE_LUA)
  return()
endif()

add_definitions(-DSOL_SAFE_USERTYPE -DSOL_CHECK_ARGUMENTS -ftemplate-depth=1500)

choreonoid_add_lua_module(LuaBody
  LuaBodyModule.cpp
  )

target_link_libraries(LuaBody
  CnoidBody
  LuaUtil
  )
