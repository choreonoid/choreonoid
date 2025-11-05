option(BUILD_OMPL_PLUGIN "Building OMPL Plugin" OFF)
if(NOT BUILD_OMPL_PLUGIN)
  return()
endif()

# The required packages can be installed by the following command in Ubuntu.
# sudo apt install libompl-dev
find_package(ompl REQUIRED)

if(MSVC)
  # Fix OMPL debug/release configuration
  if(OMPL_LIBRARIES AND NOT OMPL_LIBRARIES MATCHES "debug|optimized")
    # Split the libraries list
    set(OMPL_LIBS_TEMP ${OMPL_LIBRARIES})
    list(GET OMPL_LIBS_TEMP 0 OMPL_MAIN_LIBRARY)
    list(REMOVE_AT OMPL_LIBS_TEMP 0)
    set(OMPL_OTHER_LIBS ${OMPL_LIBS_TEMP})
    # Check if the first element is the OMPL library
    if(OMPL_MAIN_LIBRARY MATCHES "ompl\\.lib$")
      # Current OMPL library path (release version)
      set(OMPL_LIBRARY_RELEASE ${OMPL_MAIN_LIBRARY})
      # Search for debug version
      string(REPLACE "/lib/" "/debug/lib/" OMPL_LIBRARY_DEBUG_PATH ${OMPL_MAIN_LIBRARY})
      if(EXISTS ${OMPL_LIBRARY_DEBUG_PATH})
        # Reconstruct OMPL_LIBRARIES with debug/release configuration
        set(OMPL_LIBRARIES
          optimized ${OMPL_LIBRARY_RELEASE}
          debug ${OMPL_LIBRARY_DEBUG_PATH}
          ${OMPL_OTHER_LIBS})
      endif()
    endif()
  endif()
  message(STATUS "OMPL_LIBRARIES: ${OMPL_LIBRARIES}")
endif()
  
set(target CnoidOMPLPlugin)

set(sources
  OMPLPlugin.cpp
  BodyMotionPlanning.cpp
  BodyStateValidityChecker.cpp
  )

set(headers
  BodyMotionPlanning.h
  )

choreonoid_make_headers_public(${headers})

include_directories(${OMPL_INCLUDE_DIRS})

make_gettext_mofiles(${target} mofiles)
choreonoid_add_plugin(${target} ${sources} ${mofiles} HEADERS ${headers})

# Uncomment to enable debug build for this plugin only
# target_compile_options(${target} PRIVATE -g -O0 -DDEBUG -UNDEBUG)

target_link_libraries(${target} PRIVATE CnoidBodyPlugin ${OMPL_LIBRARIES})

if(MSVC)
  # Extract Boost targets from ompl::ompl target dependencies
  set(BOOST_TARGETS_TO_COPY)

  # Get Boost targets from ompl::ompl target's dependencies
  if(TARGET ompl::ompl)
    get_target_property(OMPL_INTERFACE_LIBS ompl::ompl INTERFACE_LINK_LIBRARIES)
    if(OMPL_INTERFACE_LIBS)
      foreach(lib ${OMPL_INTERFACE_LIBS})
        if(lib MATCHES "^Boost::")
          list(APPEND BOOST_TARGETS_TO_COPY ${lib})
        endif()
      endforeach()
    endif()
  endif()

  # Remove duplicates
  if(BOOST_TARGETS_TO_COPY)
    list(REMOVE_DUPLICATES BOOST_TARGETS_TO_COPY)
  endif()

  # Install DLLs for each Boost target
  foreach(boost_target ${BOOST_TARGETS_TO_COPY})
    if(TARGET ${boost_target})
      # Get release DLL
      get_target_property(boost_dll_release ${boost_target} IMPORTED_LOCATION_RELEASE)
      if(NOT boost_dll_release)
        get_target_property(boost_dll_release ${boost_target} IMPORTED_LOCATION)
      endif()
      
      # Get debug DLL
      get_target_property(boost_dll_debug ${boost_target} IMPORTED_LOCATION_DEBUG)
      
      # Install DLLs
      if(boost_dll_release AND EXISTS ${boost_dll_release})
        install(FILES ${boost_dll_release}
          DESTINATION bin
          CONFIGURATIONS Release RelWithDebInfo MinSizeRel
          )
      endif()
      message(STATUS "boost_dll_release: ${boost_dll_release}")

      if(boost_dll_debug AND EXISTS ${boost_dll_debug})
        install(FILES ${boost_dll_debug}
          DESTINATION bin
          CONFIGURATIONS Debug
        )
      endif()
      message(STATUS "boost_dll_debug: ${boost_dll_debug}")
    endif()
  endforeach()
endif()
