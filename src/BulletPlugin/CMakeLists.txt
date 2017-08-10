option(BUILD_BULLET_PLUGIN "Building BulletPlugin" OFF)

if(NOT BUILD_BULLET_PLUGIN)
  return()
endif()

#set(CMAKE_BUILD_TYPE Debug)

set(BULLET_DIR ${BULLET_DIR} CACHE PATH "set the top directory of the Bullet Physics library")
if(UNIX)
  if(NOT BULLET_DIR)
    PKG_CHECK_MODULES(bullet REQUIRED bullet)
    # HACD is not included in bullet.pc, we thus have to add it ourselves.
    LIST(APPEND ${bullet_LIBRARIES} HACD)
  else()
    find_package(Bullet PATHS ${BULLET_DIR}/lib/cmake/bullet NO_DEFAULT_PATH)
    if(BULLET_FOUND)
      set(bullet_VERSION ${BULLET_VERSION_STRING})
      set(bullet_INCLUDE_DIRS ${BULLET_DIR}/${BULLET_INCLUDE_DIRS} )
      set(bullet_LIBRARY_DIRS ${BULLET_DIR}/${BULLET_LIBRARY_DIRS} )
      set(bullet_LIBRARIES ${BULLET_LIBRARIES} HACD)
    else()
      set(bullet_INCLUDE_DIRS ${BULLET_DIR}/include/bullet)
      set(bullet_LIBRARY_DIRS ${BULLET_DIR}/lib/)
      set(bullet_LIBRARIES BulletDynamics BulletCollision LinearMath HACD)
    endif()
  endif()
elseif(MSVC)
  if(NOT BULLET_DIR)
    message(FATAL_ERROR "Please specify the directory of the Bullet Physics library to BULLET_DIR.")
  endif()
  if(BULLET_DIR)
    set(bullet_LIBRARIES optimized BulletCollision optimized BulletDynamics optimized LinearMath optimized HACD
                         debug BulletCollision_Debug debug BulletDynamics_Debug debug LinearMath_Debug debug HACD_Debug)
    set(bullet_INCLUDE_DIRS ${BULLET_DIR}/include/bullet ${BULLET_DIR}/src ${BULLET_DIR}/Extras)
    set(bullet_LIBRARY_DIRS ${BULLET_DIR}/lib/)
    add_definitions(-DBT_VER_GT_281 -DBT_USE_DOUBLE_PRECISION)
  endif()
endif()

add_definitions(${bullet_CFLAGS})

if(UNIX)
#  message ("bullet version " ${bullet_VERSION})
  if(${bullet_VERSION} VERSION_GREATER 2.84)
    add_definitions(-DBT_VER_GT_284)
  endif()
  if(${bullet_VERSION} VERSION_GREATER 2.81)
    add_definitions(-DBT_VER_GT_281)
  endif()
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

make_gettext_mofiles(${target} mofiles)
add_cnoid_plugin(${target} SHARED ${sources} ${headers} ${mofiles})
target_link_libraries(${target} CnoidBodyPlugin ${bullet_LIBRARIES})
apply_common_setting_for_plugin(${target} "${headers}")
install_external_libraries(${BULLET_DIR}/bin ${BULLET_DIR}/lib ${bullet_LIBRARIES})

if(ENABLE_PYTHON)
  if(USE_PYBIND11)
    add_subdirectory(pybind11)
  elseif(USE_BOOST_PYTHON)
    add_subdirectory(boostpython)
  endif()
endif()
