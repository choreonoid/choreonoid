# Detect ODE availability to decide the default value of BUILD_ODE_PLUGIN.
# The user can still override this by specifying BUILD_ODE_PLUGIN explicitly.
# The detection results are reused below to avoid probing ODE twice.
set(DEFAULT_BUILD_ODE_PLUGIN OFF)
if(ODE_DIR)
  set(DEFAULT_BUILD_ODE_PLUGIN ON)
elseif(UNIX)
  pkg_check_modules(ODE QUIET ode)
  if(ODE_FOUND)
    set(DEFAULT_BUILD_ODE_PLUGIN ON)
  endif()
elseif(MSVC)
  find_library(ode_lib ode_double
    HINTS "c:/Program Files/ODE/lib" "c:/local/ODE/lib" NO_CACHE)
  if(ode_lib)
    get_filename_component(ode_lib_dir ${ode_lib} DIRECTORY)
    get_filename_component(ODE_DIR ${ode_lib_dir} DIRECTORY)
    set(ODE_DIR ${ODE_DIR} CACHE PATH "set the top directory of Open Dynamics Engine" FORCE)
    set(DEFAULT_BUILD_ODE_PLUGIN ON)
  endif()
endif()
option(BUILD_ODE_PLUGIN "Building ODEPlugin" ${DEFAULT_BUILD_ODE_PLUGIN})
option(USE_ODE_STATIC_LIB "ODE Plugin uses a static library." OFF)

if(NOT BUILD_ODE_PLUGIN)
  return()
endif()

set(ODE_DIR ${ODE_DIR} CACHE PATH "set the top directory of Open Dynamics Engine")

if(UNIX)
  if(NOT ODE_DIR)
    if(NOT ODE_FOUND)
      pkg_check_modules(ODE REQUIRED ode)
    endif()
    if(USE_ODE_STATIC_LIB)
      set(ODE_LIBRARIES ode.a)
    else()
      set(ODE_LIBRARIES ode)
    endif()
  endif()
elseif(MSVC)
  if(NOT ODE_DIR)
    find_library(ode_lib ode_double HINTS "c:/Program Files/ODE/lib" "c:/local/ODE/lib" NO_CACHE)
    if(ode_lib)
      get_filename_component(ode_lib_dir ${ode_lib} DIRECTORY)
      get_filename_component(ODE_DIR ${ode_lib_dir} DIRECTORY)
      set(ODE_DIR ${ODE_DIR} CACHE PATH "set the top directory of Open Dynamics Engine" FORCE)
    endif()
  endif()
  if(NOT ODE_DIR)
    message(FATAL_ERROR "Please specify the directory of Open Dynamics Engine to ODE_DIR.")
  endif()
endif()

if(ODE_DIR)
  set(ODE_INCLUDE_DIRS ${ODE_DIR}/include)

  if(UNIX)
    set(ODE_LIBRARY_DIRS ${ODE_DIR}/lib)
    include_directories(${ODE_INCLUDE_DIRS})
    link_directories(${ODE_LIBRARY_DIRS})
    set(ODE_LIBRARIES ode.a)

  elseif(MSVC)
    set(ODE_LIBRARY_DIRS ${ODE_DIR}/lib)
    set(ODE_LIBRARIES optimized ode_double debug ode_doubled)
    include_directories(${ODE_INCLUDE_DIRS})
    link_directories(${ODE_LIBRARY_DIRS})

    if(INSTALL_RUNTIME_DEPENDENCIES)
      install(PROGRAMS ${ODE_DIR}/bin/ode_double.dll DESTINATION bin
        CONFIGURATIONS Release RelWithDebInfo MinSizeRel)
      install(PROGRAMS ${ODE_DIR}/bin/ode_doubled.dll DESTINATION bin CONFIGURATIONS Debug)

      if(INSTALL_SDK_WITH_EXTLIBS)
        install(FILES ${ODE_DIR}/lib/ode_double.lib DESTINATION ${CHOREONOID_LIB_SUBDIR}
          CONFIGURATIONS Release RelWithDebInfo MinSizeRel)
        install(FILES ${ODE_DIR}/lib/ode_doubled.lib DESTINATION ${CHOREONOID_LIB_SUBDIR} CONFIGURATIONS Debug)
        install(DIRECTORY ${ODE_DIR}/include/ode DESTINATION ${CNOID_HEADER_SUBDIR})
      endif()
    endif()
  endif()
endif()

if(UNIX AND NOT APPLE)
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-Bsymbolic")
endif()

set(target CnoidODEPlugin)

set(sources
  ODEPlugin.cpp
  ODESimulatorItem.cpp
  ODECollisionDetector.cpp
)

set(headers
)

choreonoid_make_gettext_mo_files(${target} mofiles)
choreonoid_add_plugin(${target} ${sources} ${mofiles} HEADERS ${headers})
target_link_libraries(${target} PUBLIC CnoidBodyPlugin PRIVATE ${ODE_LIBRARIES})

if(ENABLE_PYTHON)
  choreonoid_add_python_binding_subdirectory()
endif()
