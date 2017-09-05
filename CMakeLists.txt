cmake_minimum_required(VERSION 2.8.10)
cmake_policy(SET CMP0003 NEW)
set(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS true)

include(FindPkgConfig)
include(CheckIncludeFiles)

project(Choreonoid)

# set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules/)

set(CNOID_MAJOR_VERSION 1) 
set(CNOID_MINOR_VERSION 6)
set(CNOID_PATCH_VERSION 0)
set(CNOID_VERSION ${CNOID_MAJOR_VERSION}.${CNOID_MINOR_VERSION})
set(CNOID_FULL_VERSION ${CNOID_MAJOR_VERSION}.${CNOID_MINOR_VERSION}.${CNOID_PATCH_VERSION})

set(CNOID_INTERNAL_VERSION 1)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)

set(CNOID_DIR ${CMAKE_INSTALL_PREFIX})
set(CNOID_SUBDIR choreonoid-${CNOID_VERSION})
set(CNOID_PLUGIN_SUBDIR lib/${CNOID_SUBDIR})

if(UNIX)
  set(cpp_standard "-std=c++11")
  include(CheckCXXCompilerFlag OPTIONAL RESULT_VARIABLE included)
  if(included)
    check_cxx_compiler_flag("-std=c++14" has_cpp14)
    if(has_cpp14)
      set(cpp_standard "-std=c++14")
    endif()
  endif()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${cpp_standard}")
  add_definitions("-pthread")
endif()

if(WIN32)
  set(CNOID_HEADER_SUBDIR "include")
  set(CNOID_SHARE_SUBDIR "share")
  set(CNOID_DOC_SUBDIR "share/doc")
else()
  set(CNOID_HEADER_SUBDIR "include/${CNOID_SUBDIR}")
  set(CNOID_SHARE_SUBDIR "share/${CNOID_SUBDIR}")
  set(CNOID_DOC_SUBDIR "share/doc/${CNOID_SUBDIR}")
endif()

set(CNOID_SHARE_DIR "${CNOID_DIR}/${CNOID_SHARE_SUBDIR}")

set(CNOID_SOURCE_SHARE_DIR "${PROJECT_SOURCE_DIR}/share")


option(ENABLE_GUI "Enable GUI components" ON)

if(WIN32)
  set(DEFAULT_INSTALL_SDK OFF)
  set(DEFAULT_INSTALL_RUNTIME_DEPENDENCIES ON)
else()
  set(DEFAULT_INSTALL_SDK ON)
  set(DEFAULT_INSTALL_RUNTIME_DEPENDENCIES OFF)
endif()

option(INSTALL_RUNTIME_DEPENDENCIES "Installing the runtimes of external libraries" ${DEFAULT_INSTALL_RUNTIME_DEPENDENCIES})
option(INSTALL_SDK "Installing SDK files such as header files" ${DEFAULT_INSTALL_SDK})
option(INSTALL_SDK_WITH_EXTLIBS "The SDK installation includes the external libraries" OFF)

if(MSVC AND CMAKE_CL_64)
   add_definitions(-D_WIN64)
endif()

if(MSVC)
  set_property(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS_DEBUG _ITERATOR_DEBUG_LEVEL=1)
endif()

function(install_external_libraries dll_dir lib_dir)

  set(libraries ${ARGV})

  list(REMOVE_AT libraries 0 1)
  if(INSTALL_RUNTIME_DEPENDENCIES AND MSVC)
    set(conf general)
    foreach(library ${libraries})
      if(library STREQUAL general)
	set(conf general)
      elseif(library STREQUAL optimized)
	set(conf optimized)
      elseif(library STREQUAL debug)
	set(conf debug)
      else()
	get_filename_component(filename ${library} NAME_WE)
	if(conf STREQUAL general)
	  if(EXISTS ${dll_dir}/${filename}.dll)
	    install(PROGRAMS ${dll_dir}/${filename}.dll DESTINATION bin)
	  endif()
	  if(EXISTS ${lib_dir}/${filename}.lib AND INSTALL_SDK_WITH_EXTLIBS)
	    install(PROGRAMS ${lib_dir}/${filename}.lib DESTINATION lib)
	  endif()
	elseif(conf STREQUAL optimized)
	  if(EXISTS ${dll_dir}/${filename}.dll)
	    install(PROGRAMS ${dll_dir}/${filename}.dll DESTINATION bin CONFIGURATIONS Release RelWithDebInfo MinSizeRel)
	  endif()
	  if(EXISTS ${lib_dir}/${filename}.lib AND INSTALL_SDK_WITH_EXTLIBS)
	    install(PROGRAMS ${lib_dir}/${filename}.lib DESTINATION lib CONFIGURATIONS Release RelWithDebInfo MinSizeRel)
	  endif()
	elseif(conf STREQUAL debug)
	  if(EXISTS ${dll_dir}/${filename}.dll)
	    install(PROGRAMS ${dll_dir}/${filename}.dll DESTINATION bin CONFIGURATIONS Debug)
	  endif()
	  if(EXISTS ${lib_dir}/${filename}.lib AND INSTALL_SDK_WITH_EXTLIBS)
	    install(PROGRAMS ${lib_dir}/${filename}.lib DESTINATION lib CONFIGURATIONS Debug)
	  endif()
	endif()
      endif()
    endforeach()
  endif()

endfunction()



if(NOT CMAKE_BUILD_TYPE)
  set(
    CMAKE_BUILD_TYPE Release CACHE STRING
    "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
    FORCE)
endif()

if(UNIX)
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -finline-functions")
  option(ENABLE_GCC_FVISIBILITY_HIDDEN "Use the -fvisibility=hidden option when the sources are compiled" ON)
endif()

set(ADDITIONAL_CXX_FLAGS_RELEASE ${ADDITIONAL_CXX_FLAGS_RELEASE} CACHE STRING "Additional c++ compiler optimization flags")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${EXTRA_CXX_FLAGS_RELEASE} ${ADDITIONAL_CXX_FLAGS_RELEASE}")


if(MSVC)

  set(ext_compiler_options "/Ob2 /Ox /Oi /Ot /Oy /GT /GS- /fp:fast")
  if(CMAKE_CL_64)
    set(MSVC_ENABLE_SSE OFF CACHE BOOL "Enable SSE instructions on VC++." FORCE)
  else()
    option(MSVC_ENABLE_SSE "Enable SSE instructions on VC++." ON)
  endif()
  if(MSVC_ENABLE_SSE)
    set(ext_compiler_options "${ext_compiler_options} /arch:SSE /arch:SSE2 /fp:fast")
  endif()
  option(MSVC_ENABLE_AVX "Enable AVX instructions on VC++." OFF)
  if(MSVC_ENABLE_AVX)
    set(ext_compiler_options "${ext_compiler_options} /arch:AVX")
  endif()
  option(MSVC_ENABLE_AVX2 "Enable AVX2 instructions on VC++." OFF)
  if(MSVC_ENABLE_AVX2)
    set(ext_compiler_options "${ext_compiler_options} /arch:AVX2")
  endif()
  set(ext_linker_options "")

  option(MSVC_ENABLE_GLOBAL_OPTIMIZATION "Global optimization with compiler option /GL and linker option /LTCG" ON)
  if(MSVC_ENABLE_GLOBAL_OPTIMIZATION)
    set(ext_compiler_options "${ext_compiler_options} /GL")
    set(ext_linker_options "${ext_linker_options} /LTCG")
  endif()

  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${ext_compiler_options}")
  set(CMAKE_MODULE_LINKER_FLAGS_RELEASE "${CMAKE_MODULE_LINKER_FLAGS_RELEASE} ${ext_linker_options}")
  set(CMAKE_SHARED_LINKER_FLAGS_RELEASE "${CMAKE_SHARED_LINKER_FLAGS_RELEASE} ${ext_linker_options}")
  set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} ${ext_linker_options}")

  if(MSVC_ENABLE_SSE)
    set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} /GS- /fp:fast /arch:SSE2 /arch:SSE2")

    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /fp:fast /arch:SSE2 /arch:SSE2")
  endif()
endif()

find_package(Threads)

if(ENABLE_GUI)
  find_package(OpenGL)
  include_directories(${OPENGL_INCLUDE_DIR})
endif()


option(ENABLE_INSTALL_RPATH "Enable RPATH setting for installed binary files" ON)

if(ENABLE_INSTALL_RPATH)
  set(CMAKE_SKIP_BUILD_RPATH FALSE)
  if(APPLE)
    set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE) 
  else()
    set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE) 
  endif()
  set(CMAKE_INSTALL_RPATH_USE_LINK_PATH FALSE)
else()
  set(CMAKE_SKIP_BUILD_RPATH FALSE)
  set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE) 
  set(CMAKE_INSTALL_RPATH "")
  set(CMAKE_INSTALL_RPATH_USE_LINK_PATH FALSE)
endif()


option(CNOID_ENABLE_BACKWARD_COMPATIBILITY "Enable some backward compatibility" OFF)
if(CNOID_ENABLE_BACKWARD_COMPATIBILITY)
  set_property(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS CNOID_BACKWARD_COMPATIBILITY)
endif()


# commands
if(UNIX)
  set(RMDIR rm -fr)
elseif(WIN32)
  set(RMDIR rmdir /S/Q)
endif()

# check "dlfcn.h" for using dlopen() and dlclose()
if(UNIX)
  check_include_files(dlfcn.h HAVE_DLFCN_H)
  if(NOT HAVE_DLFCN_H)
    message(FATAL_ERROR "Could not find dlfcn.h")
  endif()
endif()


# gettext
option(CNOID_ENABLE_GETTEXT "Enable the gettext library and translation messages for the internationalization" ON)

if(CNOID_ENABLE_GETTEXT)
  if(WIN32)
#    if(CMAKE_CL_64)
#      FIND_PROGRAM(GETTEXT_MSGFMT_EXECUTABLE msgfmt ${PROJECT_SOURCE_DIR}/thirdparty/windows64/bin)
#    else()
      FIND_PROGRAM(GETTEXT_MSGFMT_EXECUTABLE msgfmt ${PROJECT_SOURCE_DIR}/thirdparty/windows/bin)
#    endif()
  else()
    FIND_PROGRAM(GETTEXT_MSGFMT_EXECUTABLE msgfmt)
  endif()
  if(NOT GETTEXT_MSGFMT_EXECUTABLE)
    message(FATAL_ERROR "Could not find the msgfmt command and gettext cannot be enabled.")
  endif()
  if(WIN32 AND CMAKE_CL_64)
    set(GETTEXT_DIR ${PROJECT_SOURCE_DIR}/thirdparty/windows64)
  else()
    get_filename_component(GETTEXT_BINARY_DIR ${GETTEXT_MSGFMT_EXECUTABLE} PATH)
    get_filename_component(GETTEXT_DIR ${GETTEXT_BINARY_DIR} PATH)
  endif()
  set(GETTEXT_INCLUDE_DIR ${GETTEXT_DIR}/include)
  set(GETTEXT_LIBRARY_DIR ${GETTEXT_DIR}/lib)
  include_directories(${GETTEXT_INCLUDE_DIR})
  link_directories(${GETTEXT_LIBRARY_DIR})
  if(WIN32)
    set(GETTEXT_LIBRARIES intl)
  elseif(APPLE)
    set(GETTEXT_LIBRARIES intl)
    #set(GETTEXT_STATIC_LIBRARIES
    #  ${GETTEXT_LIBRARY_DIR}/libintl.a 
    #  ${GETTEXT_LIBRARY_DIR}/libiconv.a
    #  System.B 
    #  /System/Library/Frameworks/CoreFoundation.framework)
  else()
    set(GETTEXT_LIBRARIES "")
  endif()
  if(MSVC)
    #install_external_libraries(${GETTEXT_BINARY_DIR} ${GETTEXT_LIBRARY_DIR} ${GETTEXT_LIBRARIES})
    if(CMAKE_CL_64)
      install(FILES thirdparty/windows64/bin/libintl-8.dll DESTINATION bin)
    else()
      install(FILES thirdparty/windows/bin/intl.dll DESTINATION bin)
    endif()
    if(INSTALL_SDK)
      if(CMAKE_CL_64)
        install(FILES thirdparty/windows64/lib/intl.lib DESTINATION lib)
      else()
        install(FILES thirdparty/windows/lib/intl.lib DESTINATION lib)
      endif()
      install(FILES "${GETTEXT_INCLUDE_DIR}/libintl.h" DESTINATION ${CNOID_HEADER_SUBDIR})
    endif()
  endif()
else()
  set(GETTEXT_LIBRARIES "")
endif()

# Python
if(WIN32)
  option(ENABLE_PYTHON "Enable Python" OFF)
else()
  option(ENABLE_PYTHON "Enable Python" ON)
endif()

option(USE_PYBIND11 "Use the pybind11 library to make python bindings" OFF)
set(PYBIND11_DIR ${PROJECT_SOURCE_DIR}/thirdparty/pybind11 CACHE PATH "The directory of the pybind11 library")
if(USE_PYBIND11)
  set(USE_BOOST_PYTHON OFF)
else()
  set(USE_BOOST_PYTHON ON)
endif()
set(CNOID_USE_PYBIND11 ${USE_PYBIND11})
set(CNOID_USE_BOOST_PYTHON ${USE_BOOST_PYTHON})

option(USE_PYTHON3 "Use Python version 3 instead of version 2" OFF)

# for Config.h
if(USE_PYTHON3)
  set(CNOID_USE_PYTHON2 OFF)
else()
  set(CNOID_USE_PYTHON2 ON)
endif()

if(ENABLE_PYTHON)
  if(USE_PYBIND11)
    include_directories(${PYBIND11_DIR}/include)
  else()
    if(NOT USE_PYBIND11 AND USE_PYTHON3)
      message(FATAL_ERROR "USE_PYTHON3 requires to enable USE_PYBIND11.")
    endif()
  endif()
  # The following cache variables must be cleared to update the python version
  unset(PYTHON_INCLUDE_DIR CACHE)
  unset(PYTHON_LIBRARY CACHE)
  unset(PYTHON_LIBRARY_DEBUG CACHE)
  if(USE_PYTHON3)
    set(Python_ADDITIONAL_VERSIONS 3.7 3.6 3.5 3.4)
    find_package(PythonLibs 3 REQUIRED)
  else()
    find_package(PythonLibs 2 REQUIRED)
  endif()
  include_directories(${PYTHON_INCLUDE_PATH})
  set(CNOID_PYTHON_SUBDIR ${CNOID_PLUGIN_SUBDIR}/python)
  set(init_py "${PROJECT_BINARY_DIR}/${CNOID_PYTHON_SUBDIR}/cnoid/__init__.py")
  file(WRITE ${init_py} "")
  install(FILES ${init_py} DESTINATION ${CNOID_PYTHON_SUBDIR}/cnoid
    CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel)
endif()

# Lua
if(UNIX)
  option(ENABLE_LUA "Enable Lua" OFF)
endif()

if(ENABLE_LUA)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
  set(LUA_SOL2_DIR ${PROJECT_SOURCE_DIR}/thirdparty/sol2 CACHE PATH "set the directory of the Sol2 library")
  pkg_check_modules(LUA lua5.3)
  if(NOT LUA_FOUND)
    pkg_check_modules(LUA lua5.2)
  endif()
  if(LUA_FOUND)
    include_directories(${LUA_INCLUDE_DIRS})
    include_directories(${LUA_SOL2_DIR})
  endif()
  set(CNOID_LUA_SUBDIR ${CNOID_PLUGIN_SUBDIR}/lua)
endif()

# boost

# set(Boost_NO_SYSTEM_PATHS true)

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_ADDITIONAL_VERSIONS "1.42" "1.42.0" "1.43" "1.43.0" "1.44" "1.44.0" "1.45" "1.45.0" "1.46" "1.46.0" "1.46.1" "1.47" "1.47.0" "1.48" "1.48.0" "1.49.0" "1.50.0")


if(MSVC)
  find_package(Boost 1.36.0 QUIET OPTIONAL_COMPONENTS  bzip2 zlib)
endif()

set(boost_packages system filesystem program_options regex thread iostreams date_time)

if(ENABLE_PYTHON AND USE_BOOST_PYTHON)
  set(boost_packages ${boost_packages} python)
endif()

find_package(Boost 1.36.0 REQUIRED COMPONENTS ${boost_packages})

if(NOT Boost_FOUND)
  set(BOOST_ROOT ${BOOST_ROOT} CACHE PATH "set the directory of the boost root")
  set(BOOST_LIBRARYDIR ${BOOST_LIBRARYDIR} CACHE PATH "set the directory of the boost library")
  message(FATAL_ERROR "Boost cannot be found. Please specify the boost top directory to BOOST_ROOT.")
endif()

if(Boost_VERSION GREATER "104699")
  find_package(Boost 1.47.0 REQUIRED COMPONENTS chrono)
endif()

include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

set_property(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS $<$<CONFIG:Release>:BOOST_DISABLE_ASSERTS>)

install_external_libraries(${Boost_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS}
  ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} ${Boost_PROGRAM_OPTIONS_LIBRARY} ${Boost_THREAD_LIBRARY} 
  ${Boost_DATE_TIME_LIBRARY} ${Boost_REGEX_LIBRARY} ${Boost_CHRONO_LIBRARY} ${Boost_PYTHON_LIBRARY} 
  ${Boost_IOSTREAMS_LIBRARY} ${Boost_ZLIB_LIBRARY})


if(INSTALL_SDK_WITH_EXTLIBS)
  foreach(dir ${Boost_INCLUDE_DIRS})
    if(EXISTS ${dir}/boost)
      install(DIRECTORY ${dir}/boost DESTINATION ${CNOID_HEADER_SUBDIR})
    endif()
  endforeach()
endif()

if(MSVC)
  set_property(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS BOOST_ALL_DYN_LINK ${BOOST_LIB_DIAGNOSTIC})

  #MSVC 2010 problem
  if(MSVC_VERSION EQUAL 1600)
    # for VC++2010 Express Edition
    IF(NOT DEFINED CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS_NO_WARNINGS)
      SET(CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS_NO_WARNINGS ON)
    ENDIF()
  endif()
endif()

# eigen
if(NOT EIGEN_DIR)
  if(UNIX)
    pkg_check_modules(EIGEN eigen3>=3.2.7) # pybind11 requries Eigen version 3.2.7 or higher
    if(EIGEN_FOUND)
      set(EIGEN_DIR ${EIGEN_INCLUDE_DIRS})
    else()
      message(STATUS "Set Eigen 3.3.4 included in the thirdparty directory to EIGEN_DIR")
      set(EIGEN_DIR ${PROJECT_SOURCE_DIR}/thirdparty/eigen)
      set(EIGEN_INCLUDE_DIRS ${EIGEN_DIR})
    endif()
  endif()
else()
  set(EIGEN_INCLUDE_DIRS ${EIGEN_DIR})
endif()

set(EIGEN_DIR ${EIGEN_DIR} CACHE PATH "The directory of the Eigen library")

if(NOT EIGEN_INCLUDE_DIRS)
  message(FATAL_ERROR "Please specify the Eigen directory to EIGEN_DIR.")
else()
  include_directories(${EIGEN_INCLUDE_DIRS})
endif()

set_property(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS EIGEN_NO_DEBUG)
#set_property(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS EIGEN_NO_DEBUG EIGEN_DONT_ALIGN)

if(MSVC AND INSTALL_SDK_WITH_EXTLIBS)
  foreach(dir ${EIGEN_INCLUDE_DIRS})
    if(EXISTS "${EIGEN_INCLUDE_DIRS}/Eigen")
      install(DIRECTORY "${EIGEN_INCLUDE_DIRS}/Eigen" DESTINATION ${CNOID_HEADER_SUBDIR})
    endif()
    if(EXISTS "${EIGEN_INCLUDE_DIRS}/unsupported/Eigen")
      install(DIRECTORY "${EIGEN_INCLUDE_DIRS}/unsupported/Eigen" DESTINATION ${CNOID_HEADER_SUBDIR})
    endif()
  endforeach()
endif()


###############
#  Setup Qt4  #
###############
macro(setup_qt4)
  find_package(Qt4 4.7.0 REQUIRED)
  set(QT_USE_QTOPENGL TRUE)
  set(QT_USE_QTNETWORK TRUE)
  #set(QT_USE_QTTEST TRUE)
  include(${QT_USE_FILE})
  add_definitions(-DQT_NO_KEYWORDS)
  set(QT5 FALSE)

  if(MSVC)
    install_external_libraries(${QT_BINARY_DIR} ${QT_LIBRARY_DIR} ${QT_LIBRARIES})
    if(INSTALL_SDK_WITH_EXTLIBS)
      foreach(dir
          ${QT_QTCORE_INCLUDE_DIR} ${QT_QTGUI_INCLUDE_DIR} ${QT_QTOPENGL_INCLUDE_DIR} ${QT_QTNETWORK_INCLUDE_DIR})
        install(DIRECTORY ${dir} DESTINATION ${CNOID_HEADER_SUBDIR})
      endforeach()
    endif()
  endif()
endmacro()

###############
#  Setup Qt5  #
###############
macro(setup_qt5)
  if(MSVC)
    set(PROGRAMFILES "ProgramFiles(X86)")
    if(CMAKE_CL_64)
      if(MSVC_VERSION EQUAL 1700)    #VS2012
        set(CMAKE_LIBRARY_PATH "$ENV{PROGRAMFILES}/Windows\ Kits/8.0/Lib/win8/um/x64")
      elseif(MSVC_VERSION EQUAL 1800)    #VS2013
        set(CMAKE_LIBRARY_PATH "$ENV{PROGRAMFILES}/Windows\ Kits/8.1/Lib/win8/um/x64")
      endif()
    else()
      if(MSVC_VERSION EQUAL 1700) 
        set(CMAKE_LIBRARY_PATH "$ENV{PROGRAMFILES}/Windows\ Kits/8.0/Lib/win8/um/x86")
      elseif(MSVC_VERSION EQUAL 1800)
        set(CMAKE_LIBRARY_PATH "$ENV{PROGRAMFILES}/Windows\ Kits/8.1/Lib/win8/um/x86")
      endif()
    endif()
  endif()

  find_package(Qt5Core)
  find_package(Qt5Widgets)
  find_package(Qt5OpenGL)
  find_package(Qt5Network)
  set(QT5 TRUE)

  add_definitions(-DQT_NO_KEYWORDS -DQT_NO_OPENGL_ES_2)
  set(CMAKE_AUTOMOC OFF)
  set(CMAKE_INCLUDE_CURRENT_DIR ON)

  if(MSVC)
    list(APPEND QT_INST_LIBRARIES 
      optimized Qt5Core debug Qt5Cored
      optimized Qt5Gui debug Qt5Guid
      optimized Qt5OpenGL debug Qt5OpenGLd
      optimized Qt5Network debug Qt5Networkd
      optimized Qt5Widgets debug Qt5Widgetsd
      optimized libEGL debug libEGLd
      optimized libGLESv2 debug libGLESv2d
      optimized icuin51 debug icuin51d
      optimized icuuc51 debug icuuc51d
      optimized icudt51 debug icudt51d
      # for the Qt 5.3 binary package
      general icuin52 general icuuc52 general icudt52
      general icuin53 general icuuc53 general icudt53
      general icuin54 general icuuc54 general icudt54
      general icuin55 general icuuc55 general icudt55
      )

    install_external_libraries(${_qt5Core_install_prefix}/bin ${_qt5Core_install_prefix}/lib ${QT_INST_LIBRARIES})
    install(DIRECTORY ${_qt5Core_install_prefix}/plugins/platforms DESTINATION bin FILES_MATCHING PATTERN "qwindows*.dll")
    install(DIRECTORY ${_qt5Core_install_prefix}/plugins/imageformats DESTINATION bin FILES_MATCHING PATTERN "*.dll")

    if(INSTALL_SDK_WITH_EXTLIBS)
      foreach(dir
          ${Qt5Core_INCLUDE_DIRS} ${Qt5Gui_INCLUDE_DIRS} ${Qt5OpenGL_INCLUDE_DIRS} ${Qt5Network_INCLUDE_DIRS})
        install(DIRECTORY ${dir} DESTINATION ${CNOID_HEADER_SUBDIR})
      endforeach()
    endif()
  endif()
endmacro()

# qt
if(ENABLE_GUI)
  if(NOT MSVC OR MSVC_VERSION LESS 1700)
    set(DEFAULT_USE_QT5 FALSE)
  else()
    set(DEFAULT_USE_QT5 TRUE)
  endif()

  option(USE_QT5 "Use Qt5" ${DEFAULT_USE_QT5})

  if(USE_QT5)
    setup_qt5()
  else()
    setup_qt4()
  endif()
endif()


# CORBA, omniORB
option(ENABLE_CORBA "Enable CORBA related modules / plugins" OFF)

if(ENABLE_CORBA)

  if(UNIX)
    if(NOT OMNIORB_DIR)
      #pkg_check_modules(OMNIORB omniORB4)
      pkg_check_modules(OMNIORB omniDynamic4)
      if(OMNIORB_FOUND)
	set(OMNIORB_DIR ${OMNIORB_PREFIX})
      endif()
    else()
      set(OMNIORB_FOUND TRUE)
      set(OMNIORB_INCLUDE_DIRS ${OMNIORB_DIR}/include)
      set(OMNIORB_LIBRARY_DIRS ${OMNIORB_DIR}/lib)
    endif()
  elseif(MSVC)
    if(NOT OMNIORB_DIR)
      if(NOT $ENV{OMNI_ROOT} STREQUAL "")
	set(OMNIORB_DIR $ENV{OMNI_ROOT})
      endif()
    endif()
    if(OMNIORB_DIR)
      set(OMNIORB_FOUND TRUE)
      set(OMNIORB_INCLUDE_DIRS ${OMNIORB_DIR}/include)
      set(OMNIORB_LIBRARY_DIRS ${OMNIORB_DIR}/lib/x86_win32)
      set(OMNIORB_BINARY_DIR ${OMNIORB_DIR}/bin/x86_win32)
      set(OMNIORB_CFLAGS -D__WIN32__ -D__x86__ )

      file(GLOB libomniorb RELATIVE ${OMNIORB_LIBRARY_DIRS} "${OMNIORB_LIBRARY_DIRS}/omniORB???_rt.lib")
      get_filename_component(libomniorb ${libomniorb} NAME_WE)

      file(GLOB libomnithread RELATIVE ${OMNIORB_LIBRARY_DIRS} "${OMNIORB_LIBRARY_DIRS}/omnithread??_rt.lib")
      get_filename_component(libomnithread ${libomnithread} NAME_WE)

      file(GLOB libomnidynamic RELATIVE ${OMNIORB_LIBRARY_DIRS} "${OMNIORB_LIBRARY_DIRS}/omniDynamic???_rt.lib")
      get_filename_component(libomnidynamic ${libomnidynamic} NAME_WE)

      set(OMNIORB_LIBRARIES_RELEASE ${libomniorb} ${libomnithread} ${libomnidynamic})
      foreach(library ${OMNIORB_LIBRARIES_RELEASE})
	list(APPEND OMNIORB_LIBRARIES optimized ${library} debug ${library}d )
      endforeach()

      file(GLOB libomniorb RELATIVE ${OMNIORB_BINARY_DIR} "${OMNIORB_BINARY_DIR}/omniORB*_rt.dll")
      get_filename_component(libomniorb ${libomniorb} NAME_WE)

      file(GLOB libomnithread RELATIVE ${OMNIORB_BINARY_DIR} "${OMNIORB_BINARY_DIR}/omnithread*_rt.dll")
      get_filename_component(libomnithread ${libomnithread} NAME_WE)

      file(GLOB libomnidynamic RELATIVE ${OMNIORB_BINARY_DIR} "${OMNIORB_BINARY_DIR}/omniDynamic*_rt.dll")
      get_filename_component(libomnidynamic ${libomnidynamic} NAME_WE)

      set(OMNIORB_DLL_BASES ${libomniorb} ${libomnithread} ${libomnidynamic})

      if(INSTALL_RUNTIME_DEPENDENCIES)
	foreach(library ${OMNIORB_DLL_BASES})
	  install(PROGRAMS "${OMNIORB_BINARY_DIR}/${library}.dll" DESTINATION bin CONFIGURATIONS Release RelWithDebInfo MinSizeRel)
	  install(PROGRAMS "${OMNIORB_BINARY_DIR}/${library}d.dll" DESTINATION bin CONFIGURATIONS Debug)
	endforeach()
      endif()
    endif()
  endif()
  
  include_directories(${OMNIORB_INCLUDE_DIRS})
  link_directories(${OMNIORB_LIBRARY_DIRS})
  add_definitions(${OMNIORB_CFLAGS})
  
  set(OMNIORB_DIR ${OMNIORB_DIR} CACHE PATH "The top directory of omniORB")
  set(OMNIORB_CFLAGS ${OMNIORB_CFLAGS} CACHE STRING "Compile flags for omniORB")

  if(NOT OMNIORB_FOUND)
    message(FATAL_ERROR "CORBA-related modules require the omniORB library but the library is not found.")
  endif()

  function(idl_compile_cpp out_cpp_files out_header_files subdir)
  
    set(corba_src_dir ${CMAKE_CURRENT_SOURCE_DIR}/${subdir})
    set(corba_dir ${PROJECT_SOURCE_DIR}/include/cnoid/${subdir})

    set(idl_names ${ARGV})
    list(REMOVE_AT idl_names 0 1 2)
    
    set(idl_flags -C ${corba_src_dir} -bcxx -Wbh=.hh -Wbs=Sk.cpp -Wba -Wbd=DynSk.cpp -Wbkeep_inc_path -I${PROJECT_SOURCE_DIR}/include)

    foreach(idl_include_dir ${IDL_INCLUDE_DIRS})
      set(idl_flags ${idl_flags} -I${idl_include_dir})
    endforeach()
    
    # copy idl files to the system include directory
    file(MAKE_DIRECTORY ${corba_dir})
    foreach(idl_name ${idl_names})
      set(idl_file ${corba_src_dir}/${idl_name}.idl)
      if(UNIX)
	add_custom_command(
	  OUTPUT ${corba_dir}/${idl_name}.idl
	  COMMAND cp ${idl_file} ${corba_dir}
	  DEPENDS ${idl_file}
	  COMMENT "Copying ${idl_name}.idl to ${corba_dir}"
	  )
      elseif(MSVC)
	file(TO_NATIVE_PATH ${corba_src_dir}/${idl_name}.idl src)
	file(TO_NATIVE_PATH ${corba_dir} dest)
	add_custom_command(
	  OUTPUT ${corba_dir}/${idl_name}.idl
	  COMMAND copy ${src} ${dest}
	  DEPENDS ${idl_file}
	  COMMENT "Copying ${idl_name}.idl to ${corba_dir}")
      endif()
      set(idl_files ${idl_files} ${corba_dir}/${idl_name}.idl)
      set(idl_cpp_files ${idl_cpp_files} ${subdir}/${idl_name}Sk.cpp ${subdir}/${idl_name}DynSk.cpp)
      set(idl_h_files ${idl_h_files} ${PROJECT_SOURCE_DIR}/include/cnoid/${subdir}/${idl_name}.hh)
    endforeach()

    # idl compile
    foreach(idl_name ${idl_names})
      if(UNIX)
	add_custom_command(
	  OUTPUT ${corba_src_dir}/${idl_name}.hh ${corba_dir}/${idl_name}.hh ${corba_src_dir}/${idl_name}DynSk.cpp ${corba_src_dir}/${idl_name}Sk.cpp
	  COMMAND omniidl ${idl_flags} ${corba_dir}/${idl_name}.idl
	  COMMAND cp ${corba_src_dir}/${idl_name}.hh ${corba_dir}
	  DEPENDS ${idl_files}
	  COMMENT "Generating the C++ stubs and skeletons of ${idl_name}.idl"
	  )
      elseif(MSVC)
	file(TO_NATIVE_PATH ${corba_src_dir}/${idl_name}.hh src)
	file(TO_NATIVE_PATH ${corba_dir} dest)
	add_custom_command(
	  OUTPUT ${corba_src_dir}/${idl_name}.hh ${corba_dir}/${idl_name}.hh ${corba_src_dir}/${idl_name}Sk.cpp ${corba_src_dir}/${idl_name}DynSk.cpp
	  COMMAND for %%A in \("${OMNIORB_DIR}/bin/x86_win32"\) do %%~sA\\omniidl ${idl_flags} ${corba_dir}/${idl_name}.idl
	  COMMAND copy ${src} ${dest}
	  DEPENDS ${idl_files}
	  COMMENT "Generating the C++ stubs and skeletons of ${idl_name}.idl"
	  )
      endif()
    endforeach()
    
    set(${out_cpp_files} ${idl_cpp_files} PARENT_SCOPE)
    set(${out_header_files} ${idl_h_files} PARENT_SCOPE)

    set_source_files_properties(${idl_cpp_files} PROPERTIES GENERATED true COMPILE_FLAGS -DOMNI_UNLOADABLE_STUBS)
    
  endfunction()

  if(ENABLE_PYTHON)
    
    install(DIRECTORY ${PROJECT_BINARY_DIR}/${CNOID_PYTHON_SUBDIR} DESTINATION ${CNOID_PLUGIN_SUBDIR}
      FILES_MATCHING PATTERN "*.py")

    function(idl_compile_python target src_subdir dest_subdir global_scope)

      set(args ${ARGV})
      list(REMOVE_AT args 0 1 2 3)
      set(is_dependencies FALSE)
      foreach(arg ${args})
	if(arg STREQUAL "DEPENDS")
	  set(is_dependencies TRUE)
	else()
	  if(is_dependencies)
	    set(dependencies ${dependencies} ${arg})
	  else()
	    set(idl_names ${idl_names} ${arg})
	  endif()
	endif()
      endforeach()

      set(package "")
      set(path ${dest_subdir})
      set(dir "dummy")
      while(path AND dir)
	get_filename_component(dir ${path} NAME)
	if(dir)
	  if(package)
	    set(package "${dir}.${package}")
	  else()
	    set(package ${dir})
	  endif()
	endif()
	get_filename_component(path ${path} PATH)
      endwhile()

      set(python_dir ${PROJECT_BINARY_DIR}/${CNOID_PYTHON_SUBDIR})
      set(output_dir ${python_dir}/${dest_subdir})
      file(MAKE_DIRECTORY ${output_dir})

      set(idl_flags -bpython -Wbglobal=${global_scope} -Wbpackage=${package} -I${PROJECT_SOURCE_DIR}/include)
      foreach(idl_include_dir ${IDL_INCLUDE_DIRS})
	set(idl_flags ${idl_flags} -I${idl_include_dir})
      endforeach()
    
      foreach(idl_name ${idl_names})
	set(idl_files ${idl_files} ${PROJECT_SOURCE_DIR}/include/cnoid/corba/${src_subdir}/${idl_name}.idl)
	set(outputs ${outputs} ${output_dir}/${idl_name}_idl.py)
      endforeach()

      set(prev_output)
      foreach(idl_name ${idl_names})
	set(idl_file ${PROJECT_SOURCE_DIR}/include/cnoid/corba/${src_subdir}/${idl_name}.idl)
	if(UNIX)
	  add_custom_command(
	    OUTPUT ${output_dir}/${idl_name}_idl.py
	    COMMAND omniidl ${idl_flags} ${idl_file}
	    DEPENDS ${idl_files} ${dependencies} ${prev_output} # prev_output is necessary to make the compile sequential
	    WORKING_DIRECTORY ${python_dir}
	    )
	elseif(MSVC)
	  add_custom_command(
	    OUTPUT ${output_dir}/${idl_name}_idl.py
	    COMMAND for %%A in \("${PYTHON_INCLUDE_PATH}/../bin/x86_win32"\) do %%~sA\\omniidl ${idl_flags} ${idl_file}
	    DEPENDS ${idl_files} ${dependencies}
	    WORKING_DIRECTORY ${python_dir}
	    )
	endif()
	set(prev_output ${output_dir}/${idl_name}_idl.py)
      endforeach()

      add_custom_target(${target} ALL DEPENDS ${outputs})

    endfunction()
  endif()

endif(ENABLE_CORBA)


# doxygen
# find_package(Doxygen)

set_property(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS_DEBUG CNOID_DEBUG)

if(UNIX)
  
  option(PUT_ALL_WARNINGS "put all warnings in compile" OFF)
  if(PUT_ALL_WARNINGS)
    list(APPEND CMAKE_C_FLAGS "-Wall")
    list(APPEND CMAKE_CXX_FLAGS "-Wall")
  endif()

  option(CHECK_UNRESOLVED_SYMBOLS "check unresolved symbols in the object files when creating shared libraries" OFF)
  #mark_as_advanced(CHECK_UNRESOLVED_SYMBOLS)
  if(CHECK_UNRESOLVED_SYMBOLS)
    if(NOT APPLE)
      set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--unresolved-symbols=ignore-in-shared-libs -Wl,--warn-unresolved-symbols")
    endif()
  endif()


  # The RTTI such as dynamic_cast cannot work well with this option in the older GCC like that of Ubuntu 10.04.
  # So currently this option should be only enabled for modules which may cause symbol conficts
  #set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-Bsymbolic") 

elseif(MSVC)
  set_property(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS _CRT_SECURE_NO_WARNINGS)
  set_property(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS NOMINMAX _USE_MATH_DEFINES)
  #list(APPEND CMAKE_CXX_FLAGS "/wd4250 /wd4251 /wd4275")
endif()

include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/include)

link_directories(${PROJECT_BINARY_DIR}/lib)
link_directories(${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR})

#Simulation profiling
option(ENABLE_SIMULATION_PROFILING "Enable simulation plofiling" OFF)
if(ENABLE_SIMULATION_PROFILING)
    add_definitions(-DENABLE_SIMULATION_PROFILING)
endif()

# Document installaiton
install(FILES NEWS DESTINATION ${CNOID_DOC_SUBDIR})
install(FILES LICENSE DESTINATION ${CNOID_DOC_SUBDIR})


if(MSVC)
  if(CMAKE_CL_64)
    include_directories(${PROJECT_SOURCE_DIR}/thirdparty/windows64/include)
    link_directories(${PROJECT_SOURCE_DIR}/thirdparty/windows64/lib)
  else()
    include_directories(${PROJECT_SOURCE_DIR}/thirdparty/windows/include)
    link_directories(${PROJECT_SOURCE_DIR}/thirdparty/windows/lib)
  endif()
endif()


function(make_header_public)
  set(header_file ${ARGV0})
  if(ARGC EQUAL 1)
    get_filename_component(header ${header_file} NAME_WE)
  else()
    set(header ${ARGV1})
  endif()
  file(RELATIVE_PATH header_path ${PROJECT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/${header_file})
  file(WRITE
    "${PROJECT_SOURCE_DIR}/include/cnoid/${header}"
    "#include \"${header_path}\"\n")
endfunction()

function(make_headers_public)
  foreach(header_file ${ARGV})
    make_header_public(${header_file})
  endforeach()
endfunction()


function(add_cnoid_library)

  add_library(${ARGV})

  if(ENABLE_GCC_FVISIBILITY_HIDDEN)
    get_target_property(compile_flags ${ARGV0} COMPILE_FLAGS)
    if(NOT compile_flags)
      set(compile_flags "")
    endif()
    if(ARGV1 STREQUAL "STATIC")
      set(compile_flags "${compile_flags} -fPIC")
    endif()
    set_target_properties(${ARGV0} PROPERTIES COMPILE_FLAGS "${compile_flags} -fvisibility=hidden")
  endif()

  set_target_properties(${ARGV0} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib
    ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib
    RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)

  if(ENABLE_INSTALL_RPATH)
    if(APPLE)
      set_target_properties(${ARGV0} PROPERTIES INSTALL_NAME_DIR "@rpath")
      set_target_properties(${ARGV0} PROPERTIES LINK_FLAGS "-Wl,-rpath,@loader_path")
    else()
      set_target_properties(${ARGV0} PROPERTIES INSTALL_RPATH "$ORIGIN")
    endif()
  else()
    if(APPLE)
      set_target_properties(${ARGV0} PROPERTIES INSTALL_NAME_DIR "")
    endif()      
  endif()

endfunction()


function(add_cnoid_plugin)

  add_library(${ARGV})

  if(ENABLE_GCC_FVISIBILITY_HIDDEN)
    get_target_property(compile_flags ${ARGV0} COMPILE_FLAGS)
    if(compile_flags)
      set_target_properties(${ARGV0} PROPERTIES COMPILE_FLAGS "${compile_flags} -fvisibility=hidden")
    else()
      set_target_properties(${ARGV0} PROPERTIES COMPILE_FLAGS "-fvisibility=hidden")
    endif()
  endif()

  set_target_properties(${ARGV0} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR}
    ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR}
    RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR})

  if(ENABLE_INSTALL_RPATH)
    if(APPLE)
      set_target_properties(${ARGV0} PROPERTIES INSTALL_NAME_DIR "@rpath")
      set_target_properties(${ARGV0} PROPERTIES LINK_FLAGS "-Wl,-rpath,@loader_path,-rpath,@loader_path/..")
    else()
      set_target_properties(${ARGV0} PROPERTIES INSTALL_RPATH "$ORIGIN:$ORIGIN/..")
    endif()
  else()
    if(APPLE)
      set_target_properties(${ARGV0} PROPERTIES INSTALL_NAME_DIR "")
    else()
      set_target_properties(${ARGV0} PROPERTIES INSTALL_RPATH "$ORIGIN")
    endif()
  endif()

endfunction()


function(add_cnoid_executable)

  add_executable(${ARGV})

  set_target_properties(${ARGV0} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib
    ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib
    RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)

  if(ENABLE_INSTALL_RPATH)
    if(APPLE)
      set_target_properties(${ARGV0} PROPERTIES LINK_FLAGS "-Wl,-rpath,@executable_path/../lib")
    elseif(UNIX)
      set_target_properties(${ARGV0} PROPERTIES INSTALL_RPATH "$ORIGIN/../lib")
    endif()
  endif()

  apply_common_setting_for_target(${ARGV0})

  install(TARGETS ${ARGV0} RUNTIME DESTINATION bin CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel)

endfunction()


function(apply_common_setting_for_target target)

  set(headers ${ARGV1})

  if(MSVC)
    if(MSVC_IDE)
      if(headers)
        source_group("Header Files" FILES ${headers})
      endif()
      #set_target_properties(${target} PROPERTIES PREFIX "../" IMPORT_PREFIX "../")
    endif()

    get_target_property(existing_compile_flags ${target} COMPILE_FLAGS)
    if(existing_compile_flags STREQUAL existing_compile_flags-NOTFOUND)
      set(existing_compile_flags "")
    endif()
    set_target_properties(${target} PROPERTIES COMPILE_FLAGS "${existing_compile_flags} /MP /wd4250 /wd4251 /wd4275 /wd4819 /wd4800 /wd4018 /wd4244 /wd4267")

    get_target_property(existing_link_flags ${target} LINK_FLAGS)
    if(existing_link_flags STREQUAL existing_link_flags-NOTFOUND)
      set(existing_link_flags "")
    endif()
    set_target_properties(${target} PROPERTIES LINK_FLAGS "${existing_link_flags} /NODEFAULTLIB:LIBCMT")

    set_target_properties(${target} PROPERTIES DEBUG_POSTFIX d)
  endif()

endfunction()

function(apply_common_setting_for_library_core target)

  set(headers ${ARGV1})
  set_target_properties(${target} PROPERTIES VERSION ${CNOID_VERSION})
  apply_common_setting_for_target(${target} "${headers}")

  get_target_property(target_type ${target} TYPE)
  if(target_type STREQUAL STATIC_LIBRARY)
    if(INSTALL_SDK)
      install(TARGETS ${target}
        LIBRARY DESTINATION lib CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel
        ARCHIVE DESTINATION lib CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel)
    endif()
  else()
    install(TARGETS ${target}
      RUNTIME DESTINATION bin CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel
      LIBRARY DESTINATION lib CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel)
  endif()

endfunction()


# (target "header1 header2 header3 ...")
function(apply_common_setting_for_library target)
  apply_common_setting_for_library_core(${ARGV})
  if(INSTALL_SDK)
    set(headers ${ARGV1})
    if(headers)
      file(RELATIVE_PATH rel_src_dir ${PROJECT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
      install(FILES ${headers} DESTINATION ${CNOID_HEADER_SUBDIR}/cnoid/${rel_src_dir})
    endif()
  endif()
endfunction()


function(apply_common_setting_for_ext_library)
  apply_common_setting_for_library_core(${ARGV})
endfunction()


function(apply_common_setting_for_plugin target)

  set(headers ${ARGV1})
  apply_common_setting_for_target(${target} "${headers}")

  if(INSTALL_SDK)
    install(TARGETS ${target}
      RUNTIME DESTINATION ${CNOID_PLUGIN_SUBDIR} CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel
      LIBRARY DESTINATION ${CNOID_PLUGIN_SUBDIR} CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel
      ARCHIVE DESTINATION lib CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel)
    if(headers)
      file(RELATIVE_PATH header_subdir ${PROJECT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
      install(FILES ${headers} DESTINATION ${CNOID_HEADER_SUBDIR}/cnoid/${header_subdir})
    endif()
  else()
    install(TARGETS ${target}
      RUNTIME DESTINATION ${CNOID_PLUGIN_SUBDIR} CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel
      LIBRARY DESTINATION ${CNOID_PLUGIN_SUBDIR} CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel)
  endif()

endfunction()


if(ENABLE_PYTHON)
  function(add_cnoid_python_module)
  
    set(target ${ARGV0})
    string(REGEX REPLACE "^Py(.+)$" "\\1" module ${target})
    set(sources ${ARGV})
    list(REMOVE_AT sources 0)

    add_library(${target} SHARED ${sources})
    
    if(MSVC)
      set_target_properties(${target} PROPERTIES SUFFIX .pyd)
    else()
      if(USE_BOOST_PYTHON)
	set_target_properties(${target}  PROPERTIES
	  COMPILE_DEFINITIONS "BOOST_PYTHON_USE_GCC_SYMBOL_VISIBILITY" )
      endif()
      if(USE_PYBIND11)
	if(ENABLE_GCC_FVISIBILITY_HIDDEN)
	  get_target_property(compile_flags ${target} COMPILE_FLAGS)
	  if(compile_flags)
	    set_target_properties(${target} PROPERTIES COMPILE_FLAGS "${compile_flags} -fvisibility=hidden")
	  else()
	    set_target_properties(${target} PROPERTIES COMPILE_FLAGS "-fvisibility=hidden")
	  endif()
	endif()
	if(NOT ${CMAKE_BUILD_TYPE} MATCHES Debug)
	  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
	    target_compile_options(${target} PRIVATE "-flto;-fno-fat-lto-objects")
	    set(link_lto_flags "-flto")
	  elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	    target_compile_options(${target} PRIVATE "-flto=thin")
	    set(link_lto_flags "-flto=thin")
	    target_link_libraries(${target} PRIVATE "-flto=thin")
	  endif()
	  get_target_property(existing_link_flags ${target} LINK_FLAGS)
	  if(existing_link_flags STREQUAL existing_link_flags-NOTFOUND)
	    set(existing_link_flags "")
	  endif()
	  set_target_properties(${target} PROPERTIES LINK_FLAGS "${existing_link_flags} ${link_lto_flags}")

          #add_custom_command(TARGET ${target} POST_BUILD COMMAND strip $<TARGET_FILE:${target}>)
	endif()
      endif()
    endif()

    set_target_properties(${target}  PROPERTIES
      RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CNOID_PYTHON_SUBDIR}/cnoid
      LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CNOID_PYTHON_SUBDIR}/cnoid
      RUNTIME_OUTPUT_NAME ${module}
      LIBRARY_OUTPUT_NAME ${module}
      PREFIX "")

    if(ENABLE_INSTALL_RPATH)
      if(APPLE)
	set_target_properties(${target} PROPERTIES INSTALL_NAME_DIR "@rpath")
	set_target_properties(${target} PROPERTIES LINK_FLAGS "-Wl,-rpath,@loader_path")
      else()
	set_target_properties(${target} PROPERTIES INSTALL_RPATH "$ORIGIN:$ORIGIN/../..:$ORIGIN/../../..")
      endif()
    else()
      if(APPLE)
	set_target_properties(${target} PROPERTIES INSTALL_NAME_DIR "")
      endif()
    endif()

    install(TARGETS ${target}
      RUNTIME DESTINATION ${CNOID_PYTHON_SUBDIR}/cnoid CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel
      LIBRARY DESTINATION ${CNOID_PYTHON_SUBDIR}/cnoid CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel)
  endfunction()

  function(apply_common_setting_for_python_module target)
    set(headers ${ARGV1})
    apply_common_setting_for_target(${target} "${headers}")
    if(INSTALL_SDK)
      if(MSVC)
	install(TARGETS ${target}
	  ARCHIVE DESTINATION lib CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel)
      endif()
      if(headers)
	file(RELATIVE_PATH rel_src_dir ${PROJECT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
	install(FILES ${headers} DESTINATION ${CNOID_HEADER_SUBDIR}/cnoid/${rel_src_dir})
      endif()
    endif()
  endfunction()
endif()


if(ENABLE_LUA)
  function(add_cnoid_lua_module)
    set(target ${ARGV0})
    string(REGEX REPLACE "^Lua(.+)$" "\\1" module ${target})
    set(sources ${ARGV})
    list(REMOVE_AT sources 0)

    add_library(${target} SHARED ${sources})

    if(ENABLE_GCC_FVISIBILITY_HIDDEN)
      get_target_property(compile_flags ${ARGV0} COMPILE_FLAGS)
      if(compile_flags)
	set_target_properties(${ARGV0} PROPERTIES COMPILE_FLAGS "${compile_flags} -fvisibility=hidden")
      else()
	set_target_properties(${ARGV0} PROPERTIES COMPILE_FLAGS "-fvisibility=hidden")
      endif()
    endif()
    
    set_target_properties(${target}  PROPERTIES
      RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CNOID_LUA_SUBDIR}/cnoid
      LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CNOID_LUA_SUBDIR}/cnoid
      RUNTIME_OUTPUT_NAME ${module}
      LIBRARY_OUTPUT_NAME ${module}
      PREFIX "")

    install(TARGETS ${target}
      RUNTIME DESTINATION ${CNOID_LUA_SUBDIR}/cnoid CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel
      LIBRARY DESTINATION ${CNOID_LUA_SUBDIR}/cnoid CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel)
  endfunction()

  function(apply_common_setting_for_lua_module target)
    set(headers ${ARGV1})
    set_target_properties(${target} PROPERTIES VERSION ${CNOID_VERSION})
    apply_common_setting_for_target(${target} "${headers}")

    if(INSTALL_SDK)
      if(MSVC)
	install(TARGETS ${target}
	  ARCHIVE DESTINATION lib CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel)
      endif()
      if(headers)
	file(RELATIVE_PATH rel_src_dir ${PROJECT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
	install(FILES ${headers} DESTINATION ${CNOID_HEADER_SUBDIR}/cnoid/${rel_src_dir})
      endif()
    endif()
  endfunction()
    

endif()



#function(make_gettext_mofiles out_mofiles)
function(make_gettext_mofiles target out_mofiles)

  configure_file(${PROJECT_SOURCE_DIR}/src/Util/gettext.h.in ${CMAKE_CURRENT_SOURCE_DIR}/gettext.h)
 
  set(${out_mofiles} "")

  if(NOT CNOID_ENABLE_GETTEXT)
    return()
  endif()

  file(GLOB pofiles ${CMAKE_CURRENT_SOURCE_DIR}/po/*.po)

  foreach(pofile ${pofiles})
    get_filename_component(lang ${pofile} NAME_WE)
    set(message_location share/locale/${lang}/LC_MESSAGES)
    #file(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/${message_location})
    #set(mofile ${PROJECT_BINARY_DIR}/${message_location}/${target}-${CNOID_VERSION}.mo)
    file(MAKE_DIRECTORY ${PROJECT_SOURCE_DIR}/${message_location})
    set(mofile ${PROJECT_SOURCE_DIR}/${message_location}/${target}-${CNOID_VERSION}.mo)
    add_custom_command(
      OUTPUT ${mofile}
      COMMAND ${GETTEXT_MSGFMT_EXECUTABLE} -o ${mofile} ${pofile}
      DEPENDS ${pofile}
      )
    list(APPEND mofiles ${mofile})
    install(FILES ${mofile} DESTINATION "share/locale/${lang}/LC_MESSAGES")
  endforeach()

  set(${out_mofiles} ${mofiles} PARENT_SCOPE)

endfunction()

# libyaml
if(UNIX)
  set(USE_EXTERNAL_YAML_DEFAULT ON)
elseif(MSVC)
  set(USE_EXTERNAL_YAML_DEFAULT OFF)
endif()
option(USE_EXTERNAL_YAML "Use the yaml library installed in an external directory" ${USE_EXTERNAL_YAML_DEFAULT})

if(USE_EXTERNAL_YAML)
  set(LIBYAML_DIR CACHE PATH "set the top directory of the libyaml")
  if(LIBYAML_DIR)
    include_directories(${LIBYAML_DIR}/include)
    link_directories(${LIBYAML_DIR}/lib)
  endif()
else()
  add_subdirectory(thirdparty/yaml-0.1.7)
  include_directories(${PROJECT_SOURCE_DIR}/thirdparty/yaml-0.1.7/include)
endif()

if(UNIX)
  # png
  find_package(PNG REQUIRED)
  include_directories(${PNG_INCLUDE_DIR})
  # jpeg
  find_package(JPEG REQUIRED)
  include_directories(${JPEG_INCLUDE_DIR})
  
elseif(MSVC)
  set(PNG_LIBRARY libpng)
  set(JPEG_LIBRARY jpeg)
  include_directories(${PROJECT_SOURCE_DIR}/thirdparty/lpng1232)
  include_directories(${PROJECT_SOURCE_DIR}/thirdparty/Jpeg-6b)
  include_directories(${PROJECT_SOURCE_DIR}/thirdparty/zlib123)
  add_subdirectory(thirdparty/lpng1232)
  add_subdirectory(thirdparty/Jpeg-6b)
  add_subdirectory(thirdparty/zlib123)
endif()

add_subdirectory(src)
add_subdirectory(include)

option(ENABLE_SAMPLES "Enable samples in the sample directory" ON)
if(ENABLE_SAMPLES)
  add_subdirectory(sample)
endif()

option(ENABLE_EXT "Enable components in the ext directory" ON)
if(ENABLE_EXT)
  add_subdirectory(ext)
  # additional ext directories
  set(ADDITIONAL_EXT_DIRECTORIES ${ADDITIONAL_EXT_DIRECTORIES} CACHE FILEPATH "Additional ext directories")
  if(ADDITIONAL_EXT_DIRECTORIES)
    foreach(dir ${ADDITIONAL_EXT_DIRECTORIES})
      if(EXISTS ${dir}/CMakeLists.txt)
	add_subdirectory(${dir})
      endif()
    endforeach()
  endif()
endif()



configure_file(Doxyfile.in ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile @ONLY)

if(EXISTS ${PROJECT_SOURCE_DIR}/test)
  if(EXISTS ${PROJECT_SOURCE_DIR}/test/CMakeLists.txt)
    add_subdirectory(test)
  endif()
endif()


# installing share directory files
# This must be written not in share directory but in this top CMakeLists.txt
# to prevent the share directory from being created in the binary directory

set(subdirs model project motion script)
foreach(subdir ${subdirs})
  get_filename_component(path share/${subdir} ABSOLUTE)
  if((EXISTS ${path}) AND (IS_DIRECTORY ${path}))
    install(DIRECTORY share/${subdir} DESTINATION ${CNOID_SHARE_SUBDIR})
  endif()
endforeach()



# CPack
include(InstallRequiredSystemLibraries)

# InstallRequiredSystemLibraries does not properly support MSVS 14 yet, so do it manually.  (CMake3.5)
if(INSTALL_RUNTIME_DEPENDENCIES)
unset(CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS_DEBUG)
unset(CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS_RELEASE)
if(DEFINED MSVC_VERSION AND NOT MSVC_VERSION LESS 1900)
    # Internal: Architecture-appropriate library directory names.
    if("${CMAKE_VS_PLATFORM_NAME}" STREQUAL "ARM")
        set(_winsdk_arch8 arm) # what the WDK for Win8+ calls this architecture
    else()
        if(CMAKE_SIZEOF_VOID_P MATCHES "8")
            set(_winsdk_arch8 x64) # what the WDK for Win8+ calls this architecture
        else()
            set(_winsdk_arch8 x86) # what the WDK for Win8+ calls this architecture
        endif()
    endif()

    # The CRT is distributed with MSVS.
    get_filename_component(MSVS_DIR
        "[HKEY_LOCAL_MACHINE\\SOFTWARE\\Microsoft\\VisualStudio\\14.0;ShellFolder]" ABSOLUTE)

    # As of MSVC 19 the CRT depends on the 'univeral' CRT (which is part of Windows development kit 10 and above).
    get_filename_component(WINDOWS_KIT_DIR
        "[HKEY_LOCAL_MACHINE\\SOFTWARE\\Microsoft\\Windows Kits\\Installed Roots;KitsRoot10]" ABSOLUTE)

    file(GLOB CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS_DEBUG
        "${MSVS_DIR}/VC/redist/debug_nonredist/${_winsdk_arch8}/Microsoft.VC140.DebugCRT/*.dll"
        "${WINDOWS_KIT_DIR}/Redist/ucrt/DLLs/${_winsdk_arch8}/api-ms-win-*.dll"
        "${WINDOWS_KIT_DIR}/bin/${_winsdk_arch8}/ucrt/*.dll"
    )
    file(GLOB CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS_RELEASE
        "${MSVS_DIR}/VC/redist/${_winsdk_arch8}/Microsoft.VC140.CRT/*.dll"
        "${WINDOWS_KIT_DIR}/Redist/ucrt/DLLs/${_winsdk_arch8}/*.dll"
    )

    install(PROGRAMS ${CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS_RELEASE} DESTINATION bin )
endif()
endif()

set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Choreonoid")
set(CPACK_PACKAGE_VENDOR "Shin'ichiro Nakaoka, AIST")
#set(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_CURRENT_SOURCE_DIR}/ReadMe.txt")
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
set(CPACK_PACKAGE_VERSION_MAJOR ${CNOID_MAJOR_VERSION})
set(CPACK_PACKAGE_VERSION_MINOR ${CNOID_MINOR_VERSION})
set(CPACK_PACKAGE_VERSION_PATCH ${CNOID_PATCH_VERSION})
set(CPACK_PACKAGE_INSTALL_DIRECTORY "Choreonoid ${CNOID_VERSION}")
set(CPACK_PACKAGE_INSTALL_REGISTORY_KEY "Choreonoid ${CNOID_VERSION}")

set(CPACK_GENERATOR NSIS)
if(WIN32)
  set(CPACK_GENERATOR NSIS)
  set(CPACK_PACKAGE_ICON "${CMAKE_SOURCE_DIR}\\\\src\\\\Choreonoid\\\\icon\\\\choreonoid.ico")
  set(CPACK_NSIS_INSTALLED_ICON_NAME "bin\\\\choreonoid.exe")
  set(CPACK_NSIS_DISPLAY_NAME "Choreonoid ${CNOID_VERSION}")
  set(CPACK_NSIS_CONTACT "choreonoid@m.aist.go.jp")
  set(CPACK_NSIS_MODIFY_PATH ON)
  set(CPACK_PACKAGE_EXECUTABLES "choreonoid;Choreonoid")
  set(CPACK_CREATE_DESKTOP_LINKS choreonoid)
  set(CPACK_NSIS_URL_INFO_ABOUT "http://choreonoid.org")
  
  set(CPACK_NSIS_EXTRA_INSTALL_COMMANDS "!include \\\"FileAssociation.nsh\\\"\n\\\${registerExtension} \\\"\$INSTDIR\\\\bin\\\\choreonoid.exe\\\" \\\".cnoid\\\" \\\"ChoreonoidProject\\\" \nSystem::Call 'shell32.dll::SHChangeNotify(i, i, i, i) v (0x08000000, 0, 0, 0)'")
  set(CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS "\\\${unregisterExtension} \\\".cnoid\\\" \\\"ChoreonoidProject\\\" \nSystem::Call 'shell32.dll::SHChangeNotify(i, i, i, i) v (0x08000000, 0, 0, 0)'")

elseif(UNIX)
  set(CPACK_GENERATOR DEB)
  set(CPACK_PACKAGE_FILE_NAME "choreonoid_${CNOID_FULL_VERSION}_i386")
  set(CPACK_DEBIAN_PACKAGE_DEPENDS "libc6, libgcc1, libstdc++6, libpng12-0, libjpeg62, libyaml-0-2, zlib1g, libboost-filesystem1.42.0, libboost-date-time1.42.0, libboost-program-options1.42.0, libboost-python1.42.0, libboost-regex1.42.0, libboost-system1.42.0, libboost-thread1.42.0, libqtcore4, libqtgui4, libqt4-test, libqt4-opengl")
  if(INSTALL_SDK)
    set(CPACK_DEBIAN_PACKAGE_DEPENDS "${CPACK_DEBIAN_PACKAGE_DEPENDS}, libstdc++-dev, libboost-dev, libboost-program-options-dev, libboost-python-dev, libqt4-dev, libqt4-opengl-dev")
  endif()
  set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Shin'ichiro Nakaoka")
endif()

#set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY 1)

include(CPack)

add_subdirectory(misc)
