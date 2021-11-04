macro(choreonoid_find_qt_package)

  if(${ARGC} EQUAL 0)
    set(components Core Gui Widgets Network)
  else()
    set(components ${ARGV})
  endif()

  if(UNIX)
    cmake_policy(PUSH)
    if(NOT POLICY CMP0057)
      message(FATAL_ERROR "Choreonoid requires the IN_LIST operator")
    endif()
    cmake_policy(SET CMP0057 NEW)
    if("Gui" IN_LIST components)
      list(APPEND components X11Extras)
    endif()
    cmake_policy(POP)

  elseif(MSVC)

    list(APPEND qt_msvc_vers 2015)
    if(MSVC_VERSION GREATER_EQUAL 1910) # VC++2017 or later
      list(PREPEND qt_msvc_vers 2017)
    endif()
    if(MSVC_VERSION GREATER_EQUAL 1920) # VC++2019 or later
      list(PREPEND qt_msvc_vers 2019)
    endif()
    if(MSVC_VERSION GREATER_EQUAL 1930) # VC++2022 or later
      list(PREPEND qt_msvc_vers 2022)
    endif()
    
    file(GLOB qt_dirs "c:/Qt/5.??.?")
    list(REVERSE qt_dirs)
    foreach(qt_msvc_ver ${qt_msvc_vers})
      foreach(qt_dir ${qt_dirs})
	set(qt_cmake_dir "${qt_dir}/msvc${qt_msvc_ver}_64/lib/cmake")
	if(EXISTS ${qt_cmake_dir})
	  list(APPEND qt_cmake_dirs ${qt_cmake_dir})
	endif()
      endforeach()
    endforeach()
    if(NOT qt_cmake_dirs)
      file(GLOB qt_cmake_dirs "?:/Qt/5.??.?/msvc20??_64/lib/cmake")
      list(REVERSE qt_cmake_dirs)
    endif()
  endif()

  set(CHOREONOID_QT_COMPILE_DEFINITIONS QT_NO_KEYWORDS QT_NO_OPENGL_ES_2)

  find_package(Qt5 REQUIRED COMPONENTS ${components} HINTS ${qt_cmake_dirs})
  foreach(component ${components})
    list(APPEND libs "Qt5::${component}")
  endforeach()
  set(CHOREONOID_QT_LIBRARIES ${libs})
  
endmacro()
