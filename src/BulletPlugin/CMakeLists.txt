
option(BUILD_BULLET_PLUGIN "Building BulletPlugin" OFF)

if(NOT BUILD_BULLET_PLUGIN)
  return()
endif()

#set(CMAKE_BUILD_TYPE Debug)

set(BULLET_DIR ${BULLET_DIR} CACHE PATH "set the top directory of the Bullet Physics library")
if(UNIX)
  if(NOT BULLET_DIR)
    PKG_CHECK_MODULES(bullet REQUIRED bullet)
  else()
    set(bullet_INCLUDE_DIRS ${BULLET_DIR}/include/bullet)
    set(bullet_LIBRARY_DIRS ${BULLET_DIR}/lib/)
  endif()
  # BulletConfig.cmake of version 2.82-r2704 provides a wrong name for BulletCollision,
  # so BULLET_LIBRARIES must be overwritten even if the variable is obtained from BulletConfig.cmake
  set(bullet_LIBRARIES BulletDynamics BulletCollision LinearMath HACD)

elseif(MSVC)
  if(NOT BULLET_DIR)
    message(FATAL_ERROR "Please specify the directory of the Bullet Physics library to BULLET_DIR.")
  endif()
  if(BULLET_DIR)
    set(bullet_LIBRARIES optimized BulletCollision optimized BulletDynamics optimized LinearMath optimized HACD
                         debug BulletCollision_Debug debug BulletDynamics_Debug debug LinearMath_Debug debug HACD_Debug)
    set(bullet_INCLUDE_DIRS ${BULLET_DIR}/include/bullet ${BULLET_DIR}/src ${BULLET_DIR}/Extras)
    set(bullet_LIBRARY_DIRS ${BULLET_DIR}/lib/)
  endif()
endif()

add_definitions(${bullet_CFLAGS})

if(${bullet_VERSION} VERSION_GREATER 2.82)
  add_definitions(-DBT_VER_GT_282)
endif()

include_directories(${bullet_INCLUDE_DIRS})
link_directories(${bullet_LIBRARY_DIRS})

set(target CnoidBulletPlugin)

if(MSVC)
  link_directories(${bullet_LIBRARY_DIRS}/Release)
endif()

set(sources
  BulletPlugin.cpp
  BulletSimulatorItem.cpp
  BulletCollisionDetector.cpp
  )

set(headers
  )

QT4_WRAP_CPP(sources
  )

make_gettext_mofiles(${target} mofiles)
add_cnoid_plugin(${target} SHARED ${sources} ${headers} ${mofiles})
target_link_libraries(${target} CnoidBodyPlugin ${bullet_LIBRARIES})
apply_common_setting_for_plugin(${target} "${headers}")
install_external_libraries(${BULLET_DIR}/bin ${BULLET_DIR}/lib ${bullet_LIBRARIES})

if(ENABLE_PYTHON)
  add_subdirectory(python)
endif()
