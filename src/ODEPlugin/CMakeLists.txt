option(BUILD_ODE_PLUGIN "Building ODEPlugin" OFF)
# option(BUILD_GAZEBO_ODE_PLUGIN "Building Gazebo ODE Plugin" OFF)
option(USE_ODE_STATIC_LIB "Gazebo ODE Plugin and ODE Plugin use a static library." OFF)

if(NOT BUILD_ODE_PLUGIN AND NOT BUILD_GAZEBO_ODE_PLUGIN)
  return()
endif()

if(BUILD_ODE_PLUGIN)
  set(ode ODE)
endif()
if(BUILD_GAZEBO_ODE_PLUGIN)
  set(gazebo_ode GAZEBO_ODE)
endif()
set(versions ${ode} ${gazebo_ode})

if(BUILD_ODE_PLUGIN)
  set(ODE_DIR ${ODE_DIR} CACHE PATH "set the top directory of Open Dynamics Engine")

  if(UNIX)
    if(NOT ODE_DIR)
      pkg_check_modules(ODE REQUIRED ode)
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
	  install(FILES ${ODE_DIR}/lib/ode_double.lib DESTINATION lib
	    CONFIGURATIONS Release RelWithDebInfo MinSizeRel)
	  install(FILES ${ODE_DIR}/lib/ode_doubled.lib DESTINATION lib CONFIGURATIONS Debug)
	  install(DIRECTORY ${ODE_DIR}/include/ode DESTINATION ${CNOID_HEADER_SUBDIR})
        endif()
      endif()
    endif()
  endif()
endif()

if(BUILD_GAZEBO_ODE_PLUGIN)
  set(GAZEBO_ODE_DIR ${GAZEBO_ODE_DIR} CACHE PATH "set the top directory of Gazebo Open Dynamics Engine")
  
  if(UNIX)
    if(NOT GAZEBO_ODE_DIR)
      pkg_check_modules(GAZEBO_ODE REQUIRED gazebo_ode)
      include_directories(${GAZEBO_ODE_INCLUDE_DIRS})
      link_directories(${GAZEBO_ODE_LIBRARY_DIRS})
      if(USE_ODE_STATIC_LIB)
        set(GAZEBO_ODE_LIBRARIES gazebo_ode.a
                        gazebo_opcode.a
                        gazebo_gimpact.a
                        gazebo_opende_ou.a
                        gazebo_ccd.a)
      else()
        set(GAZEBO_ODE_LIBRARIES gazebo_ode)
      endif()
    else()
      include(${GAZEBO_ODE_DIR}/lib/cmake/gazebo/gazebo-config-version.cmake)
      if(PACKAGE_VERSION MATCHES "^([0-9]+)\\.")
        set(GAZEBO_VERSION_MAJOR "${CMAKE_MATCH_1}")
      endif()
      include_directories(${GAZEBO_ODE_DIR}/include/gazebo-${GAZEBO_VERSION_MAJOR})
      link_directories(${GAZEBO_ODE_DIR}/lib)
      if(USE_ODE_STATIC_LIB)
        set(GAZEBO_ODE_LIBRARIES gazebo_ode.a
                        gazebo_opcode.a
                        gazebo_gimpact.a
                        gazebo_opende_ou.a
                        gazebo_ccd.a)
      else()
        set(GAZEBO_ODE_LIBRARIES gazebo_ode)
      endif()
    endif()
  elseif(MSVC)
  endif()
  
endif()

if(UNIX AND NOT APPLE)
  # Is this necessary to use both plugins for version 3.0 and 3.1 at the same time ?
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-Bsymbolic")
endif()

foreach(version ${versions})
 
  set(target Cnoid${version}Plugin)

  set(sources
    ODEPlugin.cpp
    ODESimulatorItem.cpp
    ODECollisionDetector.cpp
  )

  set(headers
  )

  choreonoid_make_gettext_mo_files(${target} mofiles) 
  choreonoid_add_plugin(${target} ${sources} ${mofiles} HEADERS ${headers})
  set_target_properties(${target} PROPERTIES COMPILE_DEFINITIONS ${version})
  if(${version} STREQUAL "ODE")
    target_link_libraries(${target} PUBLIC CnoidBodyPlugin PRIVATE ${ODE_LIBRARIES})
  else()
    target_link_libraries(${target} PUBLIC CnoidBodyPlugin PRIVATE ${GAZEBO_ODE_LIBRARIES})
  endif()
endforeach()

if(ENABLE_PYTHON)
  add_subdirectory(pybind11)
endif()
