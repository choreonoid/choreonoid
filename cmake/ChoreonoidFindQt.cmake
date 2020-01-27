macro(choreonoid_find_qt_package)

  if(${ARGC} EQUAL 0)
    set(components Core Gui Widgets Network)
  else()
    set(components ${ARGV})
  endif()

  if(UNIX)
    if("Gui" IN_LIST components)
      list(APPEND components X11Extras)
    endif()
  elseif(MSVC)
    file(GLOB qt_hint_dirs "c:/Qt/5.??.?/msvc20??_??/lib/cmake" "d:/Qt/5.??.?/msvc20??_??/lib/cmake")
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
