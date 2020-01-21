if(MSVC)
  # CMAKE_CXX_STANDARD 17 is supported for MSVC since version 3.10
  # The target_link_options command is available since version 3.13
  cmake_minimum_required(VERSION 3.13.0)
else()
  # Cmake version installed in Ubuntu 16.04
  cmake_minimum_required(VERSION 3.5.0)
endif()

cmake_policy(SET CMP0003 NEW)
cmake_policy(SET CMP0057 NEW)
set(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS true)

include(FindPkgConfig)
include(CheckIncludeFiles)
include(CheckCXXCompilerFlag)

project(Choreonoid)

# set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules/)
set(CHOREONOID_VERSION_MAJOR 1)
set(CHOREONOID_VERSION_MINOR 8)
set(CHOREONOID_VERSION_PATCH 0)
set(CHOREONOID_VERSION ${CHOREONOID_VERSION_MAJOR}.${CHOREONOID_VERSION_MINOR})
set(CHOREONOID_VERSION_STRING ${CHOREONOID_VERSION}.${CHOREONOID_VERSION_PATCH})
set(CHOREONOID_VERSION_ID "0x010800")
set(CHOREONOID_INTERNAL_VERSION 1)

set(CHOREONOID_VERSION_SUBDIR choreonoid-${CHOREONOID_VERSION})
set(CHOREONOID_LIB_SUBDIR lib)
set(CHOREONOID_PLUGIN_SUBDIR ${CHOREONOID_LIB_SUBDIR}/${CHOREONOID_VERSION_SUBDIR})
set(CHOREONOID_HEADER_SUBDIR include)
set(CHOREONOID_SHARE_SUBDIR share)
set(CHOREONOID_DOC_SUBDIR share/doc)

set(CHOREONOID_CMAKE_CONFIG_SUBDIR share/choreonoid/cmake) # ROS compliant
#set(CHOREONOID_CMAKE_CONFIG_SUBDIR lib/choreonoid-${CNOID_VERSION}/cmake)

if(UNIX)
  set(CHOREONOID_HEADER_SUBDIR ${CHOREONOID_HEADER_SUBDIR}/${CHOREONOID_VERSION_SUBDIR})
  set(CHOREONOID_SHARE_SUBDIR ${CHOREONOID_SHARE_SUBDIR}/${CHOREONOID_VERSION_SUBDIR})
  set(CHOREONOID_DOC_SUBDIR ${CHOREONOID_DOC_SUBDIR}/${CHOREONOID_VERSION_SUBDIR})
endif()

set(CHOREONOID_SHARE_DIR ${CMAKE_INSTALL_PREFIX}/${CHOREONOID_SHARE_SUBDIR})
set(CHOREONOID_SOURCE_SHARE_DIR ${PROJECT_SOURCE_DIR}/share)

if(PROJECT_BINARY_DIR STREQUAL PROJECT_SOURCE_DIR)
  set(CHOREONOID_BINARY_SHARE_DIR ${CHOREONOID_SOURCE_SHARE_DIR})
else()
  set(CHOREONOID_BINARY_SHARE_DIR ${PROJECT_BINARY_DIR}/${CHOREONOID_SHARE_SUBDIR})
endif()

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)

if(WIN32)
  set(DEFAULT_INSTALL_SDK OFF)
  set(DEFAULT_INSTALL_RUNTIME_DEPENDENCIES ON)
else()
  set(DEFAULT_INSTALL_SDK ON)
  set(DEFAULT_INSTALL_RUNTIME_DEPENDENCIES OFF)
endif()

option(INSTALL_RUNTIME_DEPENDENCIES "Installing the runtimes of external libraries" ${DEFAULT_INSTALL_RUNTIME_DEPENDENCIES})
option(INSTALL_SDK "Installing SDK files such as header files" ${DEFAULT_INSTALL_SDK})
if(MSVC)
  option(INSTALL_SDK_WITH_EXTLIBS "The SDK installation includes the external libraries" OFF)
endif()
set(CHOREONOID_SDK_INCLUDE_DIRS ${CMAKE_INSTALL_PREFIX}/${CHOREONOID_HEADER_SUBDIR})
set(CHOREONOID_INSTALL_SDK ${INSTALL_SDK})

set(CNOID_MAJOR_VERSION ${CHOREONOID_VERSION_MAJOR}) 
set(CNOID_MINOR_VERSION ${CHOREONOID_VERSION_MINOR})
set(CNOID_PATCH_VERSION ${CHOREONOID_VERSION_PATCH})
set(CNOID_VERSION ${CHOREONOID_VERSION})
set(CNOID_FULL_VERSION ${CHOREONOID_VERSION_STRING})
set(CNOID_VERSION_ID ${CHOREONOID_VERSION_ID})
set(CNOID_INTERNAL_VERSION ${CHOREONOID_INTERNAL_VERSION})
set(CNOID_VERSION_SUBDIR ${CHOREONOID_VERSION_SUBDIR})
set(CNOID_PLUGIN_SUBDIR ${CHOREONOID_PLUGIN_SUBDIR})
set(CNOID_HEADER_SUBDIR ${CHOREONOID_HEADER_SUBDIR})
set(CNOID_SHARE_SUBDIR ${CHOREONOID_SHARE_SUBDIR})
set(CNOID_DOC_SUBDIR ${CHOREONOID_DOC_SUBDIR})
set(CNOID_SHARE_DIR ${CHOREONOID_SHARE_DIR})
set(CNOID_SOURCE_SHARE_DIR ${CHOREONOID_SOURCE_SHARE_DIR})
set(CNOID_BINARY_SHARE_DIR ${CHOREONOID_BINARY_SHARE_DIR})
set(CNOID_SDK_INCLUDE_DIRS ${CHOREONOID_SDK_INCLUDE_DIRS})

# C++ version
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_VERSION VERSION_LESS 3.8.0) # CMake 3.8 or later is required to enable C++17
  option(ENABLE_CXX_STANDARD_17 "Enable C++ 17 when it is available to the compiler" OFF)
endif()
if(NOT ENABLE_CXX_STANDARD_17)
  unset(has_cpp17 CACHE)
endif()
if(UNIX)
  # Old GCC does not implement the regex library correctly
  if(CMAKE_COMPILER_IS_GNUCC AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 4.9.3)
    messag(FATAL_ERROR "The version of GCC is too old to build Choreonoid")
  endif()
  if(ENABLE_CXX_STANDARD_17)
    check_cxx_compiler_flag("-std=c++17" has_cpp17)
    if(has_cpp17)
      set(CMAKE_CXX_STANDARD 17)
    endif()
  endif()
  if(NOT has_cpp17)
    check_cxx_compiler_flag("-std=c++14" has_cpp14)
    if(has_cpp14)
      set(CMAKE_CXX_STANDARD 14)
    endif()
  endif()
  if(CMAKE_COMPILER_IS_GNUCC)
    # To allow for reloading a controller shared library
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --no-gnu-unique")
  endif()
  add_definitions("-pthread")

elseif(MSVC)
  if(ENABLE_CXX_STANDARD_17)
    check_cxx_compiler_flag("/std:c++17" has_cpp17)
    if(has_cpp17)
      set(CMAKE_CXX_STANDARD 17)
    endif()
  endif()
  if(NOT has_cpp17)
    check_cxx_compiler_flag("/std:c++14" has_cpp14)
    if(has_cpp14)
      set(CMAKE_CXX_STANDARD 14)
    endif()
  endif()
  if(${MSVC_VERSION} GREATER_EQUAL 1910)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Zc:__cplusplus")
  endif()
endif()

if(MSVC AND CMAKE_CL_64)
  add_definitions(-D_WIN64)
endif()

# The following option is not necessary.
# Instead of this, use include_directories(${CMAKE_CURRENT_BINARY_DIR}) where necessary.
# set(CMAKE_INCLUDE_CURRENT_DIR ON)

if(NOT PROJECT_BINARY_DIR STREQUAL PROJECT_SOURCE_DIR)
  include_directories(${PROJECT_BINARY_DIR})
  include_directories(${PROJECT_BINARY_DIR}/include)
endif()
include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/include)

link_directories(${PROJECT_BINARY_DIR}/lib)
link_directories(${PROJECT_BINARY_DIR}/${CHOREONOID_PLUGIN_SUBDIR})

if(NOT CMAKE_BUILD_TYPE)
  set(
    CMAKE_BUILD_TYPE Release CACHE STRING
    "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
    FORCE)
endif()

if(UNIX)
  set(CHOREONOID_DEFAULT_FVISIBILITY_HIDDEN ON)
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -finline-functions")
  option(ENABLE_NATIVE_CPU_ARCHITECTURE "Use the -march=native option to enable instruction set supported by the native processor architecture" OFF)
  if(ENABLE_NATIVE_ARCHITECTURE)
    set(compiler_general_options "-march=native")
  endif()
endif()

if(MSVC)
  set(compiler_general_options "/MP /wd4250 /wd4251 /wd4275 /wd4819 /wd4800 /wd4018 /wd4244 /wd4267")
  set(compiler_optimization_options "/O2 /Ob2 /Oi /Gy /GS- /fp:fast")

  if(${MSVC_VERSION} GREATER_EQUAL 1915)
    add_definitions(-D_ENABLE_EXTENDED_ALIGNED_STORAGE)
  endif()

  if(CMAKE_CL_64)
    option(MSVC_ENABLE_AVX "Enable AVX instructions on VC++." OFF)
    option(MSVC_ENABLE_AVX2 "Enable AVX2 instructions on VC++." OFF)
    if(MSVC_ENABLE_AVX2)
      set(compiler_general_options "${compiler_general_options} /arch:AVX2")
    elseif(MSVC_ENABLE_AVX)
      set(compiler_general_options "${compiler_general_options} /arch:AVX")
    endif()
  else()
    option(MSVC_ENABLE_SSE "Enable SSE instructions" ON)
    if(MSVC_ENABLE_SSE)
      set(compiler_general_options "${compiler_general_options} /arch:SSE /arch:SSE2")
    endif()
  endif()

  set(linker_optimization_options "")
  option(MSVC_ENABLE_GLOBAL_OPTIMIZATION "Global optimization with compiler option /GL and linker option /LTCG" OFF)
  if(MSVC_ENABLE_GLOBAL_OPTIMIZATION)
    set(compiler_optimization_options "${compiler_optimization_options} /GL")
    set(linker_optimization_options "${linker_optimization_options} /LTCG")
  endif()
endif()

set(ADDITIONAL_CXX_FLAGS ${ADDITIONAL_CXX_FLAGS} CACHE STRING "Additional c++ compiler optimization flags")

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${ADDITIONAL_CXX_FLAGS} ${compiler_optimization_options} ${compiler_general_options}")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} ${compiler_optimization_options} ${compiler_general_options}")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} ${ADDITIONAL_CXX_FLAGS} ${compiler_optimization_options} ${compiler_general_options}")
set(CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} ${compiler_optimization_options} ${compiler_general_options}")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} ${compiler_general_options}")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} ${compiler_general_options}")

set(CMAKE_STATIC_LINKER_FLAGS_RELEASE "${CMAKE_STATIC_LINKER_FLAGS_RELEASE} ${linker_optimization_options}")
set(CMAKE_MODULE_LINKER_FLAGS_RELEASE "${CMAKE_MODULE_LINKER_FLAGS_RELEASE} ${linker_optimization_options}")
set(CMAKE_SHARED_LINKER_FLAGS_RELEASE "${CMAKE_SHARED_LINKER_FLAGS_RELEASE} ${linker_optimization_options}")
set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} ${linker_optimization_options}")

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

set(CHOREONOID_ENABLE_INSTALL_RPATH true)

if(UNIX)
  option(ENABLE_NEW_DTAGS "Make LD_LIBRARY_PATH take precedence over RPATH with --enable-new-dtags" OFF)
  if(ENABLE_NEW_DTAGS)
    set(CMAKE_EXE_LINKER_FLAGS "-Wl,--enable-new-dtags")
    set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--enable-new-dtags")
  else()
    set(CMAKE_EXE_LINKER_FLAGS "-Wl,--disable-new-dtags")
    set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--disable-new-dtags")
  endif()
endif()

set(ENABLE_BACKWARD_COMPATIBILITY_DEFAULT OFF)
if(CNOID_ENABLE_BACKWARD_COMPATIBILITY) # Old option name
  set(ENABLE_BACKWARD_COMPATIBILITY_DEFAULT ${CNOID_ENABLE_BACKWARD_COMPATIBILITY})
endif()
option(ENABLE_BACKWARD_COMPATIBILITY "Enable some backward compatibility" ${ENABLE_BACKWARD_COMPATIBILITY_DEFAULT})
if(ENABLE_BACKWARD_COMPATIBILITY)
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
option(ENABLE_GETTEXT "Enable the gettext library and translation messages for the internationalization" ON)
set(CHOREONOID_ENABLE_GETTEXT ${ENABLE_GETTEXT})
if(ENABLE_GETTEXT)
  if(WIN32)
    FIND_PROGRAM(CHOREONOID_GETTEXT_MSGFMT_EXECUTABLE msgfmt ${PROJECT_SOURCE_DIR}/thirdparty/windows/bin)
  else()
    FIND_PROGRAM(CHOREONOID_GETTEXT_MSGFMT_EXECUTABLE msgfmt)
  endif()
  if(NOT CHOREONOID_GETTEXT_MSGFMT_EXECUTABLE)
    message(FATAL_ERROR "Could not find the msgfmt command and gettext cannot be enabled.")
  endif()
  if(WIN32 AND CMAKE_CL_64)
    set(GETTEXT_DIR ${PROJECT_SOURCE_DIR}/thirdparty/windows64)
  else()
    get_filename_component(GETTEXT_BINARY_DIR ${CHOREONOID_GETTEXT_MSGFMT_EXECUTABLE} PATH)
    get_filename_component(GETTEXT_DIR ${GETTEXT_BINARY_DIR} PATH)
  endif()
  set(GETTEXT_INCLUDE_DIR ${GETTEXT_DIR}/include)
  set(GETTEXT_LIBRARY_DIR ${GETTEXT_DIR}/lib)
  include_directories(${GETTEXT_INCLUDE_DIR})
  link_directories(${GETTEXT_LIBRARY_DIR})
  if(WIN32)
    set(GETTEXT_LIBRARIES intl)
  else()
    set(GETTEXT_LIBRARIES "")
  endif()
  if(MSVC)
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
      install(FILES "${GETTEXT_INCLUDE_DIR}/libintl.h" DESTINATION ${CHOREONOID_HEADER_SUBDIR})
    endif()
  endif()
else()
  set(GETTEXT_LIBRARIES "")
endif()

find_package(Threads)

option(ENABLE_GUI "Enable GUI components" ON)

# OpenGL
if(ENABLE_GUI)
  if(POLICY CMP0072)
    cmake_policy(SET CMP0072 OLD)
  endif()
  find_package(OpenGL)
  include_directories(${OPENGL_INCLUDE_DIR})
endif()

# Python
if(WIN32)
  option(ENABLE_PYTHON "Enable Python" OFF)
else()
  option(ENABLE_PYTHON "Enable Python" ON)
endif()

option(USE_PYTHON3 "Use Python version 3 instead of version 2" ON)
# for Config.h
if(USE_PYTHON3)
  set(CNOID_USE_PYTHON2 OFF)
else()
  set(CNOID_USE_PYTHON2 ON)
endif()

set(CHOREONOID_PYTHON_SUBDIR ${CHOREONOID_PLUGIN_SUBDIR}/python)
set(CNOID_PYTHON_SUBDIR ${CHOREONOID_PYTHON_SUBDIR})

if(MSVC)
  set(USE_EXTERNAL_PYBIND11_DEFAULT OFF)
else()
  set(USE_EXTERNAL_PYBIND11_DEFAULT ON)
endif()
option(USE_EXTERNAL_PYBIND11 "Use the Pybind11 library installed in the system" ${USE_EXTERNAL_PYBIND11_DEFAULT})

if(ENABLE_PYTHON)
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
  include_directories(${PYTHON_INCLUDE_DIRS})
  set(init_py "${PROJECT_BINARY_DIR}/${CHOREONOID_PYTHON_SUBDIR}/cnoid/__init__.py")
  file(WRITE ${init_py} "")
  install(FILES ${init_py} DESTINATION ${CHOREONOID_PYTHON_SUBDIR}/cnoid)
  if(MSVC)
    add_definitions(-DHAVE_SNPRINTF)
  endif()
  list(APPEND CHOREONOID_SDK_INCLUDE_DIRS ${PYTHON_INCLUDE_PATH})
  set(CHOREONOID_PYTHON_INCLUDE_PATH ${PYTHON_INCLUDE_PATH})

  if(USE_EXTERNAL_PYBIND11)
    find_package(pybind11 2.2.4 QUIET)
    if(pybind11_FOUND)
      set(PYBIND11_INCLUDE_DIRS ${pybind11_INCLUDE_DIRS})
      if(INSTALL_SDK_WITH_EXTLIBS)
	foreach(dir ${PYBIND11_INCLUDE_DIRS})
	  if(EXISTS ${dir}/pybind11)
            install(DIRECTORY ${dir}/pybind11 DESTINATION ${CHOREONOID_HEADER_SUBDIR})
	  endif()
	endforeach()
      else()
	list(APPEND CHOREONOID_SDK_INCLUDE_DIRS ${PYBIND11_INCLUDE_DIRS})
      endif()
    endif()
  endif()
  if(NOT USE_EXTERNAL_PYBIND11 OR NOT pybind11_FOUND)
    set(PYBIND11_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/thirdparty/pybind11-2.4.3/include)
    if(INSTALL_SDK)
      install(DIRECTORY ${PYBIND11_INCLUDE_DIRS}/pybind11 DESTINATION ${CHOREONOID_HEADER_SUBDIR})
    else()
      list(APPEND CHOREONOID_SDK_INCLUDE_DIRS ${PYBIND11_INCLUDE_DIRS})
    endif()
  endif()
  include_directories(${PYBIND11_INCLUDE_DIRS})

endif()

# This variable is kept for backward compatibility
set(USE_PYBIND11 ${ENABLE_PYTHON})

# Lua
if(UNIX)
  option(ENABLE_LUA "Enable Lua" OFF)
endif()

if(ENABLE_LUA)
  if(CMAKE_CXX_STANDARD LESS 14)
    message(FATAL_ERROR "Lua wrapper requires C++14 or later C++ versions")
  endif()
  set(LUA_SOL2_DIR ${PROJECT_SOURCE_DIR}/thirdparty/sol2 CACHE PATH "set the directory of the Sol2 library")
  pkg_check_modules(LUA lua5.3)
  if(NOT LUA_FOUND)
    pkg_check_modules(LUA lua5.2)
  endif()
  if(LUA_FOUND)
    include_directories(${LUA_INCLUDE_DIRS})
    include_directories(${LUA_SOL2_DIR})
  endif()
  set(CHOREONOID_LUA_SUBDIR ${CHOREONOID_PLUGIN_SUBDIR}/lua)
endif()

# boost
set(Boost_USE_STATIC_LIBS OFF)
set(Boost_ADDITIONAL_VERSIONS "1.62" "1.63" "1.64" "1.65" "1.66" "1.67" "1.68" "1.69" "1.70")
set(boost_packages system program_options iostreams)
if(NOT has_cpp17)
  set(boost_packages ${boost_packages} filesystem)
endif()
if(MSVC)
  find_package(Boost 1.58.0 REQUIRED COMPONENTS ${boost_packages} OPTIONAL_COMPONENTS bzip2 zlib)
else()
  find_package(Boost 1.58.0 REQUIRED COMPONENTS ${boost_packages})
endif()

include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})
set_property(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS $<$<CONFIG:Release>:BOOST_DISABLE_ASSERTS>)

if(MSVC)
  set_property(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS BOOST_ALL_DYN_LINK ${BOOST_LIB_DIAGNOSTIC})

  install_external_libraries(${Boost_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS}
    ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} ${Boost_PROGRAM_OPTIONS_LIBRARY}
    ${Boost_IOSTREAMS_LIBRARY} ${Boost_ZLIB_LIBRARY})

  if(INSTALL_SDK_WITH_EXTLIBS)
    foreach(dir ${Boost_INCLUDE_DIRS})
      if(EXISTS ${dir}/boost)
	install(DIRECTORY ${dir}/boost DESTINATION ${CHOREONOID_HEADER_SUBDIR})
      endif()
    endforeach()
  endif()
endif()

# eigen
if(MSVC)
  set(USE_EXTERNAL_EIGEN_DEFAULT OFF)
else()
  set(USE_EXTERNAL_EIGEN_DEFAULT ON)
endif()
option(USE_EXTERNAL_EIGEN "Use the Eigen library installed in the system" ${USE_EXTERNAL_EIGEN_DEFAULT})

if(USE_EXTERNAL_EIGEN)
  find_package(Eigen3 QUIET)
  if(EIGEN3_FOUND)
    if(INSTALL_SDK_WITH_EXTLIBS)
      foreach(dir ${EIGEN3_INCLUDE_DIRS})
	if(EXISTS ${dir}/Eigen)
	  install(DIRECTORY ${dir}/Eigen DESTINATION ${CHOREONOID_HEADER_SUBDIR})
	endif()
	if(EXISTS ${dir}/unsupported)
	  install(DIRECTORY ${dir}/unsupported DESTINATION ${CHOREONOID_HEADER_SUBDIR})
	endif()
      endforeach()
    else()
      list(APPEND CHOREONOID_SDK_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
    endif()
  endif()
endif()

if(NOT USE_EXTERNAL_EIGEN OR NOT EIGEN3_FOUND)
  set(EIGEN3_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/thirdparty/eigen-3.2.10)
  if(INSTALL_SDK)
    install(DIRECTORY ${EIGEN3_INCLUDE_DIRS}/Eigen ${EIGEN3_INCLUDE_DIRS}/unsupported
      DESTINATION ${CHOREONOID_HEADER_SUBDIR})
  else()
    list(APPEND CHOREONOID_SDK_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
  endif()
endif()

include_directories(${EIGEN3_INCLUDE_DIRS})

if(ENABLE_GUI)
  # Qt5
  set(qt5_components Core Gui Widgets Network)
  if(UNIX)
    list(APPEND qt5_components X11Extras)
  elseif(MSVC)
    file(GLOB qt_cmake_dirs "c:/Qt/5.??.?/msvc20??_??/lib/cmake" "d:/Qt/5.??.?/msvc20??_??/lib/cmake")
    list(SORT qt_cmake_dirs)
    list(REVERSE qt_cmake_dirs)
  endif()
  find_package(Qt5 COMPONENTS ${qt5_components} HINTS ${qt_cmake_dirs})
  foreach(component ${qt5_components})
    list(APPEND libs "Qt5::${component}")
  endforeach()
  set(QT_LIBRARIES ${libs})
  add_definitions(-DQT_NO_KEYWORDS -DQT_NO_OPENGL_ES_2)
  set(CMAKE_AUTOMOC OFF)

  if(MSVC)
    list(APPEND QT_INST_LIBRARIES 
      optimized Qt5Core debug Qt5Cored
      optimized Qt5Gui debug Qt5Guid
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
      unset(include_dirs)
      foreach(dir ${Qt5Core_INCLUDE_DIRS} ${Qt5Gui_INCLUDE_DIRS} ${Qt5Widgets_INCLUDE_DIRS} ${Qt5Network_INCLUDE_DIRS})
	if(NOT dir MATCHES "/include/?$")
	  list(APPEND include_dirs ${dir})
	endif()
      endforeach()
      list(REMOVE_DUPLICATES include_dirs)
      install(DIRECTORY ${include_dirs} DESTINATION ${CHOREONOID_HEADER_SUBDIR})
    endif()
  endif()

  if(NOT INSTALL_SDK_WITH_EXTLIBS)
    list(APPEND CHOREONOID_SDK_INCLUDE_DIRS
      ${Qt5Core_INCLUDE_DIRS}
      ${Qt5Gui_INCLUDE_DIRS}
      ${Qt5Widgets_INCLUDE_DIRS}
      ${Qt5OpenGL_INCLUDE_DIRS}
      ${Qt5Network_INCLUDE_DIRS})
  endif()
endif()

# Assimp
if(MSVC AND (NOT DEFINED ENABLE_ASSIMP))
  file(GLOB assimp_dirs "c:/local/Assimp*/lib/cmake/assimp-*" "d:/local/Assimp*/lib/cmake/assimp-*")
  find_package(ASSIMP QUIET HINTS ${assimp_dirs})
else()
  find_package(ASSIMP QUIET)
endif()
if(ASSIMP_FOUND)
  set(is_assimp_enabled ON)
else()
  set(is_assimp_enabled OFF)
endif()
option(ENABLE_ASSIMP "Enable Assimp functions" ${is_assimp_enabled})
if(ENABLE_ASSIMP AND (NOT ASSIMP_FOUND))
  message(FATAL_ERROR "Please specify the directory containing the CMake config files of the ASSIMP library to ASSIMP_DIR.")
endif()
if(MSVC)
  if(INSTALL_RUNTIME_DEPENDENCIES)
    foreach(library ${ASSIMP_LIBRARIES})
      install(PROGRAMS ${ASSIMP_ROOT_DIR}/bin/${library}.dll DESTINATION bin)
    endforeach()
  endif()
endif()

# CORBA, omniORB
option(ENABLE_CORBA "Enable CORBA related modules / plugins" OFF)

if(ENABLE_CORBA)
  # The OMNIDYNAMIC_XXX variables are used for storing the omniORB information
  # because the OMNIORB_XXX varialbes collide with the OpenRTM-aist configuration
  # and the omniDynamic4 library is actually required to build some modules.
  if(UNIX)
    if(NOT OMNIDYNAMIC_DIR)
      pkg_check_modules(OMNIDYNAMIC omniDynamic4)
      if(OMNIDYNAMIC_FOUND)
	set(OMNIDYNAMIC_DIR ${OMNIDYNAMIC_PREFIX})
      endif()
    else()
      set(OMNIDYNAMIC_FOUND TRUE)
      set(OMNIDYNAMIC_INCLUDE_DIRS ${OMNIDYNAMIC_DIR}/include)
      set(OMNIDYNAMIC_LIBRARY_DIRS ${OMNIDYNAMIC_DIR}/lib)
    endif()
  elseif(MSVC)
    if(NOT OMNIDYNAMIC_DIR)
      if(NOT $ENV{OMNI_ROOT} STREQUAL "")
	set(OMNIDYNAMIC_DIR $ENV{OMNI_ROOT})
      endif()
    endif()
    if(OMNIDYNAMIC_DIR)
      set(OMNIDYNAMIC_FOUND TRUE)
      set(OMNIDYNAMIC_INCLUDE_DIRS ${OMNIDYNAMIC_DIR}/include)
      set(OMNIDYNAMIC_LIBRARY_DIRS ${OMNIDYNAMIC_DIR}/lib/x86_win32)
      set(OMNIDYNAMIC_BINARY_DIR ${OMNIDYNAMIC_DIR}/bin/x86_win32)
      set(OMNIDYNAMIC_CFLAGS -D__WIN32__ -D__x86__ )

      file(GLOB libomniorb RELATIVE ${OMNIDYNAMIC_LIBRARY_DIRS} "${OMNIDYNAMIC_LIBRARY_DIRS}/omniORB???_rt.lib")
      get_filename_component(libomniorb ${libomniorb} NAME_WE)

      file(GLOB libomnithread RELATIVE ${OMNIDYNAMIC_LIBRARY_DIRS} "${OMNIDYNAMIC_LIBRARY_DIRS}/omnithread??_rt.lib")
      get_filename_component(libomnithread ${libomnithread} NAME_WE)

      file(GLOB libomnidynamic RELATIVE ${OMNIDYNAMIC_LIBRARY_DIRS} "${OMNIDYNAMIC_LIBRARY_DIRS}/omniDynamic???_rt.lib")
      get_filename_component(libomnidynamic ${libomnidynamic} NAME_WE)

      set(OMNIDYNAMIC_LIBRARIES_RELEASE ${libomniorb} ${libomnithread} ${libomnidynamic})
      foreach(library ${OMNIDYNAMIC_LIBRARIES_RELEASE})
	list(APPEND OMNIDYNAMIC_LIBRARIES optimized ${library} debug ${library}d )
      endforeach()

      file(GLOB libomniorb RELATIVE ${OMNIDYNAMIC_BINARY_DIR} "${OMNIDYNAMIC_BINARY_DIR}/omniORB*_rt.dll")
      get_filename_component(libomniorb ${libomniorb} NAME_WE)

      file(GLOB libomnithread RELATIVE ${OMNIDYNAMIC_BINARY_DIR} "${OMNIDYNAMIC_BINARY_DIR}/omnithread*_rt.dll")
      get_filename_component(libomnithread ${libomnithread} NAME_WE)

      file(GLOB libomnidynamic RELATIVE ${OMNIDYNAMIC_BINARY_DIR} "${OMNIDYNAMIC_BINARY_DIR}/omniDynamic*_rt.dll")
      get_filename_component(libomnidynamic ${libomnidynamic} NAME_WE)

      set(OMNIDYNAMIC_DLL_BASES ${libomniorb} ${libomnithread} ${libomnidynamic})

      if(INSTALL_RUNTIME_DEPENDENCIES)
	foreach(library ${OMNIDYNAMIC_DLL_BASES})
	  install(PROGRAMS "${OMNIDYNAMIC_BINARY_DIR}/${library}.dll" DESTINATION bin CONFIGURATIONS Release RelWithDebInfo MinSizeRel)
	  install(PROGRAMS "${OMNIDYNAMIC_BINARY_DIR}/${library}d.dll" DESTINATION bin CONFIGURATIONS Debug)
	endforeach()
      endif()
    endif()
  endif()
  
  include_directories(${OMNIDYNAMIC_INCLUDE_DIRS})
  link_directories(${OMNIDYNAMIC_LIBRARY_DIRS})
  add_definitions(${OMNIDYNAMIC_CFLAGS})
  
  set(OMNIDYNAMIC_DIR ${OMNIDYNAMIC_DIR} CACHE PATH "The top directory of omniORB")
  set(OMNIDYNAMIC_CFLAGS ${OMNIDYNAMIC_CFLAGS} CACHE STRING "Compile flags for omniORB")

  if(NOT OMNIDYNAMIC_FOUND)
    message(FATAL_ERROR "CORBA-related modules require the omniORB library but the library is not found.")
  endif()

  if(ENABLE_PYTHON)
    install(DIRECTORY ${PROJECT_BINARY_DIR}/${CHOREONOID_PYTHON_SUBDIR} DESTINATION ${CHOREONOID_PLUGIN_SUBDIR}
      FILES_MATCHING PATTERN "*.py")
  endif()

  set(CHOREONOID_OMNIDYNAMIC_DIR ${OMNIDYNAMIC_DIR})

endif(ENABLE_CORBA)

# OpenRTM
if(BUILD_OPENRTM_PLUGIN)
  find_package(OpenRTM REQUIRED CONFIG)

  # Check version
  if(NOT OPENRTM_VERSION VERSION_LESS "1.2.0")
    list(APPEND OPENRTM_DEFINITIONS -DOPENRTM_VERSION12)
    list(APPEND OPENRTM_DEFINITIONS -Wno-deprecated)

  elseif(OPENRTM_VERSION VERSION_EQUAL "1.1.2")
    list(APPEND OPENRTM_DEFINITIONS -DOPENRTM_VERSION11)
    # Remove extra "-I" from the elements of OPENRTM_INCLUDE_DIRS
    unset(include_dirs)
    foreach(dir ${OPENRTM_INCLUDE_DIRS})
      string(REGEX REPLACE "^-I" "" stripped ${dir})
      list(APPEND include_dirs ${stripped})
    endforeach()
    set(OPENRTM_INCLUDE_DIRS ${include_dirs})
    # Remove extra "-L" from the elements of OPENRTM_LIBRARY_DIRS
    unset(library_dirs)
    foreach(dir ${OPENRTM_LIBRARY_DIRS})
      string(REGEX REPLACE "^-L" "" stripped ${dir})
      list(APPEND library_dirs ${stripped})
    endforeach()
    set(OPENRTM_LIBRARY_DIRS ${library_dirs})

  else()
    message(FATAL_ERROR "OpenRTM version is INVALID.")
  endif()

  set(IDL_INCLUDE_DIRS ${IDL_INCLUDE_DIRS} ${OPENRTM_INCLUDE_DIRS})

  option(USE_BUILTIN_CAMERA_IMAGE_IDL "Enable this option when you run hrpsys-base on Choreonoid" OFF)
  if(USE_BUILTIN_CAMERA_IMAGE_IDL)
    list(APPEND OPENRTM_DEFINITIONS -DUSE_BUILTIN_CAMERA_IMAGE_IDL)
  else()
    if(MSVC)
      unset(camera_library)
      foreach(dir ${OPENRTM_LIBRARY_DIRS})
        file(GLOB libs "${dir}/rtmCamera*.lib")
        foreach(lib ${libs})
          if(lib MATCHES "(rtmCamera.*d)\\.lib$")
            list(APPEND camera_library debug;${CMAKE_MATCH_1})
          elseif(lib MATCHES "(rtmCamera.*)\\.lib$")
            list(APPEND camera_library optimized;${CMAKE_MATCH_1})
          endif()
        endforeach()
      endforeach()
      set(OPENRTM_CAMERA_LIBRARY ${camera_library})
    else()
      set(OPENRTM_CAMERA_LIBRARY rtmCamera)
    endif()
  endif()
endif()

set_property(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS $<$<CONFIG:Debug>:CNOID_DEBUG>)

if(UNIX)
  option(PUT_EXTRA_WARNINGS "Put extra warnings in compile" OFF)
  if(PUT_EXTRA_WARNINGS)
    set(extra_warning_options "-Wunused -Woverloaded-virtual -Wsign-compare")
    #set(extra_warning_options "-Wparentheses -Wformat=2 -Wformat-nonliteral -Wunknown-pragmas -Wunused-parameter -Wuninitialized -Winit-self -pedantic -Wconversion -Wno-variadic-macros -Wno-long-long -Wno-import -Wno-missing-braces")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall ${extra_warning_options}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall ${extra_warning_options}")
  endif()

  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wreturn-type")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wreturn-type")

  option(CHECK_UNRESOLVED_SYMBOLS "check unresolved symbols in the object files when creating shared libraries" OFF)
  if(CHECK_UNRESOLVED_SYMBOLS)
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--unresolved-symbols=ignore-in-shared-libs -Wl,--warn-unresolved-symbols")
  endif()

elseif(MSVC)
  set_property(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS _CRT_SECURE_NO_WARNINGS)
  set_property(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS NOMINMAX _USE_MATH_DEFINES)
endif()

#Simulation profiling
option(ENABLE_SIMULATION_PROFILING "Enable simulation plofiling" OFF)
if(ENABLE_SIMULATION_PROFILING)
    add_definitions(-DENABLE_SIMULATION_PROFILING)
endif()

# Document installaiton
install(FILES NEWS DESTINATION ${CHOREONOID_DOC_SUBDIR})
install(FILES LICENSE DESTINATION ${CHOREONOID_DOC_SUBDIR})

option(BUILD_DOCUMENTS "Build the API reference manual" OFF)
if(BUILD_DOCUMENTS)
  add_subdirectory(doc)
endif()

if(MSVC)
  if(CMAKE_CL_64)
    include_directories(${PROJECT_SOURCE_DIR}/thirdparty/windows64/include)
    link_directories(${PROJECT_SOURCE_DIR}/thirdparty/windows64/lib)
  else()
    include_directories(${PROJECT_SOURCE_DIR}/thirdparty/windows/include)
    link_directories(${PROJECT_SOURCE_DIR}/thirdparty/windows/lib)
  endif()
endif()

include(cmake/ChoreonoidBasicBuildFunctions.cmake)

if(ENABLE_PYTHON)
  include(cmake/ChoreonoidPythonBuildFunctions.cmake)
endif()
if(ENABLE_LUA)
  include(cmake/ChoreonoidLuaBuildFunctions.cmake)
endif()
if(ENABLE_CORBA)
  include(cmake/ChoreonoidCorbaBuildFunctions.cmake)
endif()

# libyaml
if(MSVC)
  set(USE_EXTERNAL_LIBYAML_DEFAULT OFF)
else()
  set(USE_EXTERNAL_LIBYAML_DEFAULT ON)
endif()
option(USE_EXTERNAL_LIBYAML "Use the yaml library installed in an external directory" ${USE_EXTERNAL_LIBYAML_DEFAULT})

if(USE_EXTERNAL_LIBYAML)
  pkg_check_modules(LIBYAML yaml-0.1 QUIET)
endif()

if(NOT USE_EXTERNAL_LIBYAML OR NOT LIBYAML_FOUND)
  add_subdirectory(thirdparty/yaml-0.1.7)
  set(LIBYAML_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/thirdparty/yaml-0.1.7/include)
  set(LIBYAML_LIBRARY_DIRS ${PROJECT_SOURCE_DIR}/thirdparty/yaml-0.1.7/lib)
  set(LIBYAML_LIBRARIES yaml)
endif()

# pugixml
add_subdirectory(thirdparty/pugixml-1.9)
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/pugixml-1.9/src)

if(UNIX)
  # png
  find_package(PNG REQUIRED)
  include_directories(${PNG_INCLUDE_DIR})
  # jpeg
  find_package(JPEG REQUIRED)
  include_directories(${JPEG_INCLUDE_DIR})
  
elseif(MSVC)
  # png
  find_package(PNG)
  if(PNG_FOUND)
    include_directories(${PNG_INCLUDE_DIR})
  else()
    set(PNG_LIBRARY libpng)
    include_directories(${PROJECT_SOURCE_DIR}/thirdparty/zlib123)
    add_subdirectory(thirdparty/zlib123)
    include_directories(${PROJECT_SOURCE_DIR}/thirdparty/lpng1232)
    add_subdirectory(thirdparty/lpng1232)
  endif()
  # jpeg
  find_package(JPEG)
  if(JPEG_FOUND)
    include_directories(${JPEG_INCLUDE_DIR})
  else()
    set(JPEG_LIBRARY jpeg)
    include_directories(${PROJECT_SOURCE_DIR}/thirdparty/Jpeg-9c)
    add_subdirectory(thirdparty/Jpeg-9c)
  endif()
endif()

# fmtlib
find_package(fmt QUIET)
if(NOT fmt_FOUND)
  add_subdirectory(thirdparty/fmt-5.3.0)
  # Is this necessary when building on Windows?
  include_directories(${PROJECT_SOURCE_DIR}/thirdparty/fmt-5.3.0/include)
  if(INSTALL_SDK)
    install(DIRECTORY ${PROJECT_SOURCE_DIR}/thirdparty/fmt-5.3.0/include/fmt DESTINATION ${CHOREONOID_HEADER_SUBDIR})
  endif()
endif()

add_subdirectory(src)
add_subdirectory(include)
add_subdirectory(share)

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
      else()
	file(GLOB subdirs "${dir}/*")
	list(SORT subdirs)
	foreach(subdir ${subdirs})
	  if(EXISTS ${subdir}/CMakeLists.txt)
	    add_subdirectory(${subdir})
	  endif()
	endforeach()
      endif()
    endforeach()
  endif()
endif()

if(EXISTS ${PROJECT_SOURCE_DIR}/test)
  if(EXISTS ${PROJECT_SOURCE_DIR}/test/CMakeLists.txt)
    add_subdirectory(test)
  endif()
endif()

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

add_subdirectory(cmake)
add_subdirectory(misc)
