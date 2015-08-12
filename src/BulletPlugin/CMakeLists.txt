
option(BUILD_BULLET_PLUGIN "Building BulletPlugin" OFF)

if(NOT BUILD_BULLET_PLUGIN)
  return()
endif()

#set(CMAKE_BUILD_TYPE Debug)

set(BULLET_DIR ${BULLET_DIR} CACHE PATH "set the top directory of the Bullet Physics library")
set(BULLET_CFLAGS -DBT_USE_DOUBLE_PRECISION)
if(UNIX)
  if(NOT BULLET_DIR)
    FIND_PACKAGE(Bullet REQUIRED)
  else()
    set(BULLET_INCLUDE_DIRS ${BULLET_DIR}/include/bullet)
    set(BULLET_LIBRARY_DIRS ${BULLET_DIR}/lib/)
  endif()
  # BulletConfig.cmake of version 2.82-r2704 provides a wrong name for BulletCollision,
  # so BULLET_LIBRARIES must be overwritten even if the variable is obtained from BulletConfig.cmake
  set(BULLET_LIBRARIES BulletDynamics BulletCollision LinearMath HACD)

elseif(MSVC)
  if(NOT BULLET_DIR)
    message(FATAL_ERROR "Please specify the directory of the Bullet Physics library to BULLET_DIR.")
  endif()
  if(BULLET_DIR)
    set(BULLET_LIBRARIES optimized BulletCollision optimized BulletDynamics optimized LinearMath optimized HACD
                         debug BulletCollision_Debug debug BulletDynamics_Debug debug LinearMath_Debug debug HACD_Debug)
    set(BULLET_INCLUDE_DIRS ${BULLET_DIR}/include/bullet ${BULLET_DIR}/src ${BULLET_DIR}/Extras)
    set(BULLET_LIBRARY_DIRS ${BULLET_DIR}/lib/)
  endif()
endif()

add_definitions(${BULLET_CFLAGS})
include_directories(${BULLET_INCLUDE_DIRS})
link_directories(${BULLET_LIBRARY_DIRS})

set(target CnoidBulletPlugin)

if(MSVC)
  link_directories(${BULLET_LIBRARY_DIRS}/Release)
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
target_link_libraries(${target} CnoidBodyPlugin ${BULLET_LIBRARIES})
apply_common_setting_for_plugin(${target} "${headers}")
install_external_libraries(${BULLET_DIR}/bin ${BULLET_DIR}/lib ${BULLET_LIBRARIES})

if(ENABLE_PYTHON)
  add_subdirectory(python)
endif()
