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
    set(qt_msvc_hint "msvc20??_64")
    if(MSVC_VERSION GREATER_EQUAL 1910 AND MSVC_VERSION LESS 1920) # VC++2017
      # Qt provides the msvc 2015 binary for VC++2017
      set(qt_msvc_hint "msvc2015_64") 
    elseif(MSVC_VERSION LESS 1930) # VC++2019
      set(qt_msvc_hint "msvc2019_64") 
    #elseif(MSVC_VERSION LESS 1940) # VC++2022
    #  set(qt_msvc_hint "msvc2022_64") 
    endif()
    file(GLOB qt_hint_dirs "c:/Qt/5.??.?/${qt_msvc_hint}/lib/cmake" "d:/Qt/5.??.?/${qt_msvc_hint}/lib/cmake")
    list(SORT qt_hint_dirs)
    list(REVERSE qt_hint_dirs)
  endif()

  set(CHOREONOID_QT_COMPILE_DEFINITIONS QT_NO_KEYWORDS QT_NO_OPENGL_ES_2)

  find_package(Qt5 REQUIRED COMPONENTS ${components} HINTS ${qt_hint_dirs})
  foreach(component ${components})
    list(APPEND libs "Qt5::${component}")
  endforeach()
  set(CHOREONOID_QT_LIBRARIES ${libs})
  
endmacro()
