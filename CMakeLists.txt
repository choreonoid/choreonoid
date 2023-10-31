if(MSVC)
  # Version 3.13 or later is required because the following features are used in this project.
  # * CMAKE_CXX_STANDARD 17 is supported for MSVC since version 3.10
  # * The target_link_options command is available since version 3.13
  # * The install command can specifiy a target defined in a sub directory since version 3.13
  cmake_minimum_required(VERSION 3.13)
else()
  # CMake version 3.5.1 is installed in Ubuntu 16.04.
  # (Version 3.10.2 is installed in Ubuntu 18.04.)
  cmake_minimum_required(VERSION 3.5)
endif()

cmake_policy(SET CMP0003 NEW)
cmake_policy(SET CMP0057 NEW)
set(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS true)

include(FindPkgConfig)
include(CheckIncludeFiles)
include(CheckCXXCompilerFlag)

project(Choreonoid)

# set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules/)
set(CHOREONOID_VERSION_MAJOR 2)
set(CHOREONOID_VERSION_MINOR 1)
set(CHOREONOID_VERSION_PATCH 0)
set(CHOREONOID_VERSION ${CHOREONOID_VERSION_MAJOR}.${CHOREONOID_VERSION_MINOR})
set(CHOREONOID_VERSION_STRING ${CHOREONOID_VERSION}.${CHOREONOID_VERSION_PATCH})
set(CHOREONOID_VERSION_ID "0x020100")
set(CHOREONOID_INTERNAL_VERSION 3)

set(CHOREONOID_VERSION_SUBDIR choreonoid-${CHOREONOID_VERSION})
set(CHOREONOID_BIN_SUBDIR bin)
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
set(CHOREONOID_INSTALL_SDK ${INSTALL_SDK})
set(CHOREONOID_SDK_INCLUDE_DIRS ${CMAKE_INSTALL_PREFIX}/${CHOREONOID_HEADER_SUBDIR})
set(CHOREONOID_SDK_LIBRARY_DIRS
  ${CMAKE_INSTALL_PREFIX}/${CHOREONOID_LIB_SUBDIR} ${CMAKE_INSTALL_PREFIX}/${CHOREONOID_PLUGIN_SUBDIR})

# Deprecated
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

set(CMAKE_CXX_EXTENSIONS OFF)

# C++ version
if(NOT CMAKE_CXX_STANDARD)
  set(cxx_standard 11)
  if(NOT CMAKE_VERSION VERSION_LESS 3.8.0) # CMake 3.8 or later is required to enable C++17
    if(UNIX)
      # C++17 support of GCC 7 is experimental
      if(NOT CMAKE_COMPILER_IS_GNUCC OR CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 8)
	check_cxx_compiler_flag("-std=c++17" has_cpp17)
      endif()
    elseif(MSVC)
      check_cxx_compiler_flag("/std:c++17" has_cpp17)
    endif()
    if(has_cpp17)
      set(cxx_standard 17)
    endif()
  endif()
  if(NOT has_cpp17)
    if(UNIX)
      check_cxx_compiler_flag("-std=c++14" has_cpp14)
    elseif(MSVC)
      check_cxx_compiler_flag("-std=c++14" has_cpp14)
    endif()
    if(has_cpp14)
      set(cxx_standard 14)
    endif()
  endif()
  set(CMAKE_CXX_STANDARD ${cxx_standard} CACHE STRING "C++ version number. Specify either 17, 14 or 11.")
endif()

if(UNIX)
  if(CMAKE_COMPILER_IS_GNUCC)
    # To allow for reloading a controller shared library
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --no-gnu-unique")
  endif()
  add_definitions("-pthread")

elseif(MSVC)
  if(${MSVC_VERSION} GREATER_EQUAL 1910)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Zc:__cplusplus")
  endif()
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(
    CMAKE_BUILD_TYPE Release CACHE STRING
    "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
    FORCE)
endif()

set_property(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS $<$<CONFIG:Debug>:CNOID_DEBUG>)

if(UNIX)
  set(CHOREONOID_DEFAULT_FVISIBILITY_HIDDEN ON)

  set(compiler_general_options "-Wreturn-type")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -finline-functions")

  option(ENABLE_NATIVE_CPU_ARCHITECTURE "Use the -march=native option to enable instruction set supported by the native processor architecture" OFF)
  if(ENABLE_NATIVE_CPU_ARCHITECTURE)
    set(compiler_general_options "${compiler_general_options} -march=native")
  endif()

  option(CHECK_UNRESOLVED_SYMBOLS "check unresolved symbols in the object files when creating shared libraries" OFF)
  if(CHECK_UNRESOLVED_SYMBOLS)
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--unresolved-symbols=ignore-in-shared-libs -Wl,--warn-unresolved-symbols")
  endif()

  option(PUT_EXTRA_WARNINGS "Put extra warnings in compile" OFF)
  if(PUT_EXTRA_WARNINGS)
    set(compiler_general_options "${compiler_general_options} -Wall -Wunused -Woverloaded-virtual -Wsign-compare")
    # -Wparentheses -Wformat=2 -Wformat-nonliteral -Wunknown-pragmas -Wunused-parameter -Wuninitialized -Winit-self -pedantic -Wconversion -Wno-variadic-macros -Wno-long-long -Wno-import -Wno-missing-braces
  endif()
endif()

if(MSVC)
  set(CHOREONOID_DEFAULT_FVISIBILITY_HIDDEN OFF)
  set(compiler_general_options "/MP /source-charset:utf-8 /wd4018 /wd4068 /wd4244 /wd4250 /wd4251 /wd4267 /wd4275 /wd4800 /wd4819")
  set(compiler_release_options "/O2 /Ob2 /Oi /Gy /GS- /fp:fast")

  if(${MSVC_VERSION} GREATER_EQUAL 1915)
    add_definitions(-D_ENABLE_EXTENDED_ALIGNED_STORAGE)
  endif()
  if(CMAKE_CXX_STANDARD GREATER_EQUAL 17)
    option(DISABLE_CXX17_DEPRECATION_WARNINGS "Define _SILENCE_ALL_CXX17_DEPRECATION_WARNINGS" ON)
    mark_as_advanced(DISABLE_CXX17_DEPRECATION_WARNINGS)
    if(DISABLE_CXX17_DEPRECATION_WARNINGS)
      add_definitions(-D_SILENCE_ALL_CXX17_DEPRECATION_WARNINGS)
    endif()
  endif()

  if(CMAKE_CL_64)
    add_definitions(-D_WIN64)
    option(ENABLE_MSVC_AVX "Enable AVX instructions on VC++." OFF)
    option(ENABLE_MSVC_AVX2 "Enable AVX2 instructions on VC++." OFF)
    if(ENABLE_MSVC_AVX2)
      set(compiler_general_options "${compiler_general_options} /arch:AVX2")
    elseif(ENABLE_MSVC_AVX)
      set(compiler_general_options "${compiler_general_options} /arch:AVX")
    endif()
  else()
    option(ENABLE_MSVC_SSE "Enable SSE instructions" ON)
    if(ENABLE_MSVC_SSE)
      set(compiler_general_options "${compiler_general_options} /arch:SSE /arch:SSE2")
    endif()
  endif()

  set(linker_release_options "")
  option(ENABLE_MSVC_GLOBAL_OPTIMIZATION "Global optimization with compiler option /GL and linker option /LTCG" OFF)
  if(ENABLE_MSVC_GLOBAL_OPTIMIZATION)
    set(compiler_release_options "${compiler_release_options} /GL")
    set(linker_release_options "${linker_release_options} /LTCG")
  endif()

  option(ENABLE_MSVC_DEBUG_INFO_FOR_RELEASE "Enable debug information for the relase build" OFF)
  mark_as_advanced(ENABLE_MSVC_DEBUG_INFO_FOR_RELEASE)
  if(ENABLE_MSVC_DEBUG_INFO_FOR_RELEASE)
    set(compiler_release_options "${compiler_release_options} /Zi")
    set(linker_release_options "${linker_release_options} /DEBUG /OPT:REF /OPT:ICF")
  endif()

  option(USE_SUBSYSTEM_CONSOLE "Attaching a console of Windows for debugging" OFF)
  set(CHOREONOID_USE_SUBSYSTEM_CONSOLE ${USE_SUBSYSTEM_CONSOLE})

  set_property(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS
    NOMINMAX _USE_MATH_DEFINES _CRT_SECURE_NO_WARNINGS _CRT_NONSTDC_NO_DEPRECATE)
endif()

set(ADDITIONAL_CXX_FLAGS ${ADDITIONAL_CXX_FLAGS} CACHE STRING "Additional c++ compiler optimization flags")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${compiler_general_options}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${compiler_general_options}")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${ADDITIONAL_CXX_FLAGS} ${compiler_release_options}")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} ${compiler_release_options}")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} ${ADDITIONAL_CXX_FLAGS} ${compiler_release_options}")
set(CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} ${compiler_release_options}")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG}")

set(CMAKE_STATIC_LINKER_FLAGS_RELEASE "${CMAKE_STATIC_LINKER_FLAGS_RELEASE} ${linker_release_options}")
set(CMAKE_MODULE_LINKER_FLAGS_RELEASE "${CMAKE_MODULE_LINKER_FLAGS_RELEASE} ${linker_release_options}")
set(CMAKE_SHARED_LINKER_FLAGS_RELEASE "${CMAKE_SHARED_LINKER_FLAGS_RELEASE} ${linker_release_options}")
set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} ${linker_release_options}")

if(NOT PROJECT_BINARY_DIR STREQUAL PROJECT_SOURCE_DIR)
  include_directories(${PROJECT_BINARY_DIR})
  include_directories(${PROJECT_BINARY_DIR}/include)
endif()
include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/include)

link_directories(${PROJECT_BINARY_DIR}/lib)
link_directories(${PROJECT_BINARY_DIR}/${CHOREONOID_PLUGIN_SUBDIR})

function(install_runtime_dlls dll_dir)
  if(MSVC AND INSTALL_RUNTIME_DEPENDENCIES)
    set(dllfiles ${ARGN})
    set(conf general)
    unset(optional_flag)
    foreach(file ${dllfiles})
      if(file STREQUAL general)
	set(conf general)
      elseif(file STREQUAL optimized)
	set(conf optimized)
      elseif(file STREQUAL debug)
	set(conf debug)
      elseif(file STREQUAL OPTIONAL)
	set(optional_flag OPTIONAL)
      else()
	get_filename_component(filename ${file} NAME_WE)
	if(conf STREQUAL general)
	  if(EXISTS ${dll_dir}/${filename}.dll)
	    install(PROGRAMS ${dll_dir}/${filename}.dll DESTINATION bin ${OPTIONAL_FLAG})
	  else()
	    # message(STATUS "${dll_dir}/${filename}.dll not found!")
	  endif()
	elseif(conf STREQUAL optimized)
	  if(EXISTS ${dll_dir}/${filename}.dll)
	    install(PROGRAMS ${dll_dir}/${filename}.dll
	      DESTINATION bin CONFIGURATIONS Release RelWithDebInfo MinSizeRel ${OPTIONAL_FLAG})
	  endif()
	elseif(conf STREQUAL debug)
	  if(EXISTS ${dll_dir}/${filename}.dll)
	    install(PROGRAMS ${dll_dir}/${filename}.dll
	      DESTINATION bin CONFIGURATIONS Debug ${OPTIONAL_FLAG})
	  endif()
	endif()
      endif()
    endforeach()
  endif()
endfunction()

option(ENABLE_INSTALL_RPATH "Enable RPATH for installed shared libraries" ON)
mark_as_advanced(ENABLE_INSTALL_RPATH)
set(CHOREONOID_ENABLE_INSTALL_RPATH ${ENABLE_INSTALL_RPATH})
option(ENABLE_INSTALL_RPATH_USE_LINK_PATH "Specify the value of CMAKE_INSTALL_RPATH_USE_LINK_PATH" OFF)
mark_as_advanced(ENABLE_INSTALL_RPATH_USE_LINK_PATH)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH ${ENABLE_INSTALL_RPATH_USE_LINK_PATH})

if(UNIX)
  option(ENABLE_NEW_DTAGS "Make LD_LIBRARY_PATH take precedence over RPATH with --enable-new-dtags" OFF)
  if(ENABLE_NEW_DTAGS)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--enable-new-dtags")
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--enable-new-dtags")
  else()
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--disable-new-dtags")
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--disable-new-dtags")
  endif()

  option(ENABLE_GPERFTOOLS_PROFILER "Enable the CPU profiler of the Google Performance Tools")
  if(ENABLE_GPERFTOOLS_PROFILER)
    pkg_check_modules(GPREFTOOLS_PROFILER libprofiler libtcmalloc QUIET)
    if(NOT GPREFTOOLS_PROFILER_FOUND)
      message(FATAL_ERROR "libprofiler is not found")
    endif()
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
set(GETTEXT_LIBRARIES "")
if(ENABLE_GETTEXT)
  set(use_bundled_gettext false)
  if(UNIX)
    find_program(CHOREONOID_GETTEXT_MSGFMT_EXECUTABLE msgfmt)
    find_program(CHOREONOID_GETTEXT_MSGCAT_EXECUTABLE msgcat)
    if(NOT CHOREONOID_GETTEXT_MSGFMT_EXECUTABLE OR NOT CHOREONOID_GETTEXT_MSGCAT_EXECUTABLE)
      message(FATAL_ERROR "Gettext cannot be enabled because the msgfmt and msgcat commands are not found.")
    endif()
    get_filename_component(GETTEXT_BINARY_DIR ${CHOREONOID_GETTEXT_MSGFMT_EXECUTABLE} PATH)
    get_filename_component(GETTEXT_DIR ${GETTEXT_BINARY_DIR} PATH)
  elseif(MSVC)
    set(GETTEXT_DIR "${PROJECT_SOURCE_DIR}/thirdparty/gettext-0.21")
    set(CHOREONOID_GETTEXT_MSGFMT_EXECUTABLE "${GETTEXT_DIR}/bin/msgfmt.exe")
    set(CHOREONOID_GETTEXT_MSGCAT_EXECUTABLE "${GETTEXT_DIR}/bin/msgcat.exe")
    set(use_bundled_gettext true)
  endif()
  set(GETTEXT_INCLUDE_DIR "${GETTEXT_DIR}/include")
  set(GETTEXT_LIBRARY_DIR "${GETTEXT_DIR}/lib")
  include_directories(${GETTEXT_INCLUDE_DIR})
  link_directories(${GETTEXT_LIBRARY_DIR})
  if(NOT use_bundled_gettext)
    list(APPEND CHOREONOID_SDK_INCLUDE_DIRS ${GETTEXT_INCLUDE_DIR})
    list(APPEND CHOREONOID_SDK_LIBRARY_DIRS ${GETTEXT_LIBRARY_DIR})
  endif()
  if(MSVC)
    set(GETTEXT_LIBRARIES libintl-8)
    install(FILES "${GETTEXT_LIBRARY_DIR}/libintl-8.dll" DESTINATION bin)
    if(INSTALL_SDK)
      install(FILES "${GETTEXT_LIBRARY_DIR}/libintl-8.lib" DESTINATION lib)
      install(FILES "${GETTEXT_INCLUDE_DIR}/libintl.h" DESTINATION ${CHOREONOID_HEADER_SUBDIR})
    endif()
  endif()
endif()

find_package(Threads)

option(ENABLE_GUI "Enable GUI components" ON)

# OpenGL
if(ENABLE_GUI)
  # Qt5 is using the legacy OpenGL library
  set(OpenGL_GL_PREFERENCE LEGACY)
  find_package(OpenGL)
  include_directories(${OPENGL_INCLUDE_DIR})
endif()

# Python
if(WIN32)
  option(ENABLE_PYTHON "Enable Python" OFF)
else()
  option(ENABLE_PYTHON "Enable Python" ON)
endif()

option(USE_BUNDLED_PYBIND11 "Use the Pybind11 library bundled with Choreonoid" ON)

option(USE_PYTHON3 "Use Python version 3 instead of version 2" ON)
# for Config.h
if(USE_PYTHON3)
  set(CNOID_USE_PYTHON2 OFF)
else()
  set(CNOID_USE_PYTHON2 ON)
endif()

set(CHOREONOID_PYTHON_SUBDIR ${CHOREONOID_PLUGIN_SUBDIR}/python)

if(ENABLE_PYTHON)
  # The following cache variables must be cleared to update the python version
  unset(PYTHON_INCLUDE_DIR CACHE)
  unset(PYTHON_LIBRARY CACHE)
  unset(PYTHON_LIBRARY_DEBUG CACHE)
  if(USE_PYTHON3)
    find_package(PythonLibs 3 REQUIRED)
  else()
    set(Python_ADDITIONAL_VERSIONS 2.7)
    find_package(PythonLibs 2 REQUIRED)
    if(NOT CMAKE_CXX_STANDARD LESS 17)
      message(WARNING "C++14 is used when Python2 is used.")
      set(CMAKE_CXX_STANDARD 14 CACHE STRING "C++ version number. C++14 is used when Python2 is used." FORCE)
    endif()
  endif()
  include_directories(${PYTHON_INCLUDE_DIRS})
  list(APPEND CHOREONOID_SDK_INCLUDE_DIRS ${PYTHON_INCLUDE_DIRS})
  set(init_py "${PROJECT_BINARY_DIR}/${CHOREONOID_PYTHON_SUBDIR}/cnoid/__init__.py")
  file(WRITE ${init_py} "")
  install(FILES ${init_py} DESTINATION ${CHOREONOID_PYTHON_SUBDIR}/cnoid)
  if(MSVC)
    add_definitions(-DHAVE_SNPRINTF)
  endif()

  if(NOT USE_BUNDLED_PYBIND11)
    find_package(pybind11 2.6 QUIET)
  endif()
  if(pybind11_FOUND AND NOT USE_BUNDLED_PYBIND11)
    set(PYBIND11_INCLUDE_DIRS ${pybind11_INCLUDE_DIRS})
    list(APPEND CHOREONOID_SDK_INCLUDE_DIRS ${PYBIND11_INCLUDE_DIRS})
  else()
    set(PYBIND11_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/thirdparty/pybind11-2.10.3/include)
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
  mark_as_advanced(ENABLE_LUA)
endif()

if(ENABLE_LUA)
  if(CMAKE_CXX_STANDARD LESS 14)
    message(FATAL_ERROR "Lua wrapper requires C++14 or later C++ versions")
  endif()
  set(LUA_SOL2_DIR ${PROJECT_SOURCE_DIR}/thirdparty/sol2 CACHE PATH "set the directory of the Sol2 library")

  # The required packages can be installed by the following command in Ubuntu.
  # sudo apt install lua5.3 iblua5.3-dev lua-posix
  pkg_check_modules(LUA lua5.3)
  if(NOT LUA_FOUND)
    pkg_check_modules(LUA lua5.2)
  endif()
  if(NOT LUA_FOUND)
    message(FATAL_ERROR "lua library is not found")
  else()
    include_directories(${LUA_INCLUDE_DIRS})
    include_directories(${LUA_SOL2_DIR})
  endif()
  set(CHOREONOID_LUA_SUBDIR ${CHOREONOID_PLUGIN_SUBDIR}/lua)
endif()

# boost
set(Boost_USE_STATIC_LIBS OFF)
set(Boost_ADDITIONAL_VERSIONS "1.62" "1.63" "1.64" "1.65" "1.66" "1.67" "1.68" "1.69" "1.70" "1.71" "1.72" "1.73" "1.74" "1.75")
set(boost_packages program_options iostreams system)
if(CMAKE_CXX_STANDARD LESS 17)
  set(boost_packages ${boost_packages} filesystem)
endif()
if(MSVC)
  find_package(Boost 1.58.0 REQUIRED COMPONENTS ${boost_packages} OPTIONAL_COMPONENTS bzip2 zlib)
else()
  find_package(Boost 1.58.0 REQUIRED COMPONENTS ${boost_packages})
endif()

if(CMAKE_CXX_STANDARD LESS 17)
  set(FILESYSTEM_LIBRARY ${Boost_FILESYSTEM_LIBRARY})
elseif(UNIX)
  set(FILESYSTEM_LIBRARY stdc++fs)
endif()

include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})
list(APPEND CHOREONOID_SDK_INCLUDE_DIRS ${Boost_INCLUDE_DIRS})
list(APPEND CHOREONOID_SDK_LIBRARY_DIRS ${Boost_LIBRARY_DIRS})

if(MSVC)
  set_property(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS BOOST_ALL_DYN_LINK ${BOOST_LIB_DIAGNOSTIC})
  install_runtime_dlls(${Boost_LIBRARY_DIRS}
    ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} ${Boost_PROGRAM_OPTIONS_LIBRARY}
    ${Boost_IOSTREAMS_LIBRARY} ${Boost_ZLIB_LIBRARY})
endif()

# Eigen
if(MSVC)
  set(USE_BUNDLED_EIGEN_DEFAULT ON)
else()
  set(USE_BUNDLED_EIGEN_DEFAULT OFF)
endif()
option(USE_BUNDLED_EIGEN "Use the Eigen library bundled with Choreonoid" ${USE_BUNDLED_EIGEN_DEFAULT})

if(NOT USE_BUNDLED_EIGEN)
  find_package(Eigen3 QUIET)
endif()
if(EIGEN3_FOUND AND NOT USE_BUNDLED_EIGEN)
  list(APPEND CHOREONOID_SDK_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
else()
  set(EIGEN3_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/thirdparty/eigen-3.2.10)
  if(INSTALL_SDK)
    install(DIRECTORY ${EIGEN3_INCLUDE_DIRS}/Eigen ${EIGEN3_INCLUDE_DIRS}/unsupported
      DESTINATION ${CHOREONOID_HEADER_SUBDIR})
  else()
    list(APPEND CHOREONOID_SDK_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
  endif()
endif()

include_directories(${EIGEN3_INCLUDE_DIRS})

# FreeType
if(NOT MSVC)
  option(ENABLE_FREE_TYPE "Enable FreeType" ON)
  if(ENABLE_FREE_TYPE)
    pkg_check_modules(FreeType2 freetype2 REQUIRED IMPORTED_TARGET)
  endif()
else()
  option(ENABLE_FREE_TYPE "Enable FreeType" OFF)
  if(ENABLE_FREE_TYPE)
    file(GLOB freetype_dirs
      "c:/Program Files/freetype/lib/cmake/freetype*"
      "c:/local/freetype*/lib/cmake/freetype*"
      "d:/local/freetype*/lib/cmake/freetype*")
    find_package(freetype HINTS ${freetype_dirs} REQUIRED)
    if(freetype_FOUND)
      cmake_path(GET freetype_DIR PARENT_PATH freetype_dir1)
      cmake_path(GET freetype_dir1 PARENT_PATH freetype_dir2)
      cmake_path(GET freetype_dir2 PARENT_PATH freetype_ROOT_DIR)
      message(STATUS "Detecting FreeType ${freetype_VERSION} at ${freetype_ROOT_DIR}.")
      if(false) # Enable this if you want to use the DLL version of freetype
	get_target_property(freetype_dll_release freetype IMPORTED_LOCATION_RELEASE)
	get_target_property(freetype_dll_debug freetype IMPORTED_LOCATION_DEBUG)
	install_runtime_dlls(${freetype_ROOT_DIR}/bin
	  optimized ${freetype_dll_release} debug ${freetype_dll_debug})
      endif()
    endif()
  endif()
endif()
set(CNOID_ENABLE_FREE_TYPE ${ENABLE_FREE_TYPE})

if(ENABLE_GUI)
  # Qt
  include(cmake/ChoreonoidFindQt.cmake)
  choreonoid_find_qt_package(Core Gui Widgets Network Svg)
  set(CMAKE_AUTOMOC OFF)

  if(MSVC)
    list(APPEND QT_INST_LIBRARIES
      optimized Qt5Core debug Qt5Cored
      optimized Qt5Gui debug Qt5Guid
      optimized Qt5Network debug Qt5Networkd
      optimized Qt5Widgets debug Qt5Widgetsd
      optimized Qt5Svg debug Qt5Svgd
      optimized libEGL debug libEGLd
      optimized libGLESv2 debug libGLESv2d
      optimized icuin51 debug icuin51d
      optimized icuuc51 debug icuuc51d
      optimized icudt51 debug icudt51d
      # for the Qt 5.3 binary package
      general icuin52 icuuc52 icudt52
      general icuin53 icuuc53 icudt53
      general icuin54 icuuc54 icudt54
      general icuin55 icuuc55 icudt55
      )
    # It may be better to use the windeployqt command for the following process.
    # See the Qt manual on the Windows deployment.
    install_runtime_dlls(${_qt5Core_install_prefix}/bin ${QT_INST_LIBRARIES})
    install(DIRECTORY ${_qt5Core_install_prefix}/plugins/platforms DESTINATION bin FILES_MATCHING PATTERN "qwindows*.dll")
    install(DIRECTORY ${_qt5Core_install_prefix}/plugins/imageformats DESTINATION bin FILES_MATCHING PATTERN "*.dll")
    install(DIRECTORY ${_qt5Core_install_prefix}/plugins/styles DESTINATION bin FILES_MATCHING PATTERN "*.dll")
  endif()

  function(install_qt_message_files_for_languages)
    if(MSVC)
      foreach(lang ${ARGV})
	install(
	  DIRECTORY ${_qt5Core_install_prefix}/translations
	  DESTINATION bin
	  FILES_MATCHING PATTERN "*_${lang}.qm")
      endforeach()
    endif()
  endfunction()
  # Install Japanese messages by default
  install_qt_message_files_for_languages(ja)
endif()

# Assimp
if(NOT MSVC)
  find_package(assimp)
else()
  # Try to detect assmp automatically when the initial configuration
  if((NOT DEFINED ENABLE_ASSIMP) OR ENABLE_ASSIMP)
    file(GLOB assimp_dirs
      "c:/Program Files/Assimp/lib/cmake/assimp-*"
      "c:/local/Assimp*/lib/cmake/assimp-*"
      "d:/local/Assimp*/lib/cmake/assimp-*")
    find_package(assimp HINTS ${assimp_dirs})
    if(assimp_FOUND)
      message(STATUS "Detecting Assimp ${assimp_VERSION} at ${ASSIMP_ROOT_DIR}.")
    endif()
  endif()
endif()

if(assimp_FOUND)
  set(is_assimp_enabled ON)
else()
  set(is_assimp_enabled OFF)
endif()
option(ENABLE_ASSIMP "Enable Assimp functions" ${is_assimp_enabled})

if(ENABLE_ASSIMP)
  if(NOT assimp_FOUND)
    message(FATAL_ERROR "Please specify the directory containing the CMake config files of the ASSIMP library to assimp_DIR.")
  endif()
  if(MSVC)
    if(assimp_VERSION VERSION_GREATER_EQUAL 5.1.0)
      # In this case, the value of ASSIMP_LIBRARIES is the imported library name.
      get_target_property(assimp_dll ${ASSIMP_LIBRARIES} IMPORTED_LOCATION_RELEASE)
      get_target_property(assimp_dll_debug ${ASSIMP_LIBRARIES} IMPORTED_LOCATION_DEBUG)
      install_runtime_dlls(${ASSIMP_ROOT_DIR}/bin optimized ${assimp_dll} debug ${assimp_dll_debug})
    else()
      include_directories(${ASSIMP_INCLUDE_DIRS})
      link_directories(${ASSIMP_LIBRARY_DIRS})
      install_runtime_dlls(${ASSIMP_ROOT_DIR}/bin ${ASSIMP_LIBRARIES})
    endif()
  endif()
endif()

option(ENABLE_URDF "Enable URDF functions" ON)
if(ENABLE_URDF AND ENABLE_PYTHON)
  add_subdirectory(thirdparty/xacro-1.14.10)
endif()

# CORBA, omniORB
option(ENABLE_CORBA "Enable CORBA related modules / plugins" OFF)
mark_as_advanced(ENABLE_CORBA)

if(ENABLE_CORBA)
  # Note that the OMNIORB_XXX varialbes cannot be used because
  # the variables collide with those of OpenRTM-aist configuration.
  if(UNIX)
    if(NOT CHOREONOID_OMNIORB_DIR)
      # The required package can be installed by the following command in Ubuntu.
      # sudo apt install libomniorb4-dev libcos4-dev omniidl omniorb-nameserver
      pkg_check_modules(CHOREONOID_OMNIORB omniDynamic4)
      if(CHOREONOID_OMNIORB_FOUND)
	set(CHOREONOID_OMNIORB_DIR ${CHOREONOID_OMNIORB_PREFIX})
      endif()
    else()
      set(CHOREONOID_OMNIORB_FOUND TRUE)
      set(CHOREONOID_OMNIORB_INCLUDE_DIRS ${CHOREONOID_OMNIORB_DIR}/include)
      set(CHOREONOID_OMNIORB_LIBRARY_DIRS ${CHOREONOID_OMNIORB_DIR}/lib)
    endif()
  elseif(MSVC)
    if(NOT CHOREONOID_OMNIORB_DIR)
      if(NOT $ENV{OMNI_ROOT} STREQUAL "")
	set(CHOREONOID_OMNIORB_DIR $ENV{OMNI_ROOT})
      endif()
    endif()
    if(CHOREONOID_OMNIORB_DIR)
      set(CHOREONOID_OMNIORB_FOUND TRUE)
      set(CHOREONOID_OMNIORB_INCLUDE_DIRS "${CHOREONOID_OMNIORB_DIR}/include")
      set(CHOREONOID_OMNIORB_LIBRARY_DIRS "${CHOREONOID_OMNIORB_DIR}/lib/x86_win32")
      set(CHOREONOID_OMNIORB_BINARY_DIR "${CHOREONOID_OMNIORB_DIR}/bin/x86_win32")
      set(CHOREONOID_OMNIORB_CFLAGS -D__WIN32__ -D__x86__ )

      file(GLOB libomniorb RELATIVE ${CHOREONOID_OMNIORB_LIBRARY_DIRS} "${CHOREONOID_OMNIORB_LIBRARY_DIRS}/omniORB???_rt.lib")
      get_filename_component(libomniorb ${libomniorb} NAME_WE)

      file(GLOB libomnithread RELATIVE ${CHOREONOID_OMNIORB_LIBRARY_DIRS} "${CHOREONOID_OMNIORB_LIBRARY_DIRS}/omnithread??_rt.lib")
      get_filename_component(libomnithread ${libomnithread} NAME_WE)

      file(GLOB libomnidynamic RELATIVE ${CHOREONOID_OMNIORB_LIBRARY_DIRS} "${CHOREONOID_OMNIORB_LIBRARY_DIRS}/omniDynamic???_rt.lib")
      get_filename_component(libomnidynamic ${libomnidynamic} NAME_WE)

      set(CHOREONOID_OMNIORB_LIBRARIES_RELEASE ${libomniorb} ${libomnithread} ${libomnidynamic})
      foreach(library ${CHOREONOID_OMNIORB_LIBRARIES_RELEASE})
	list(APPEND CHOREONOID_OMNIORB_LIBRARIES optimized ${library} debug ${library}d )
      endforeach()

      file(GLOB libomniorb RELATIVE ${CHOREONOID_OMNIORB_BINARY_DIR} "${CHOREONOID_OMNIORB_BINARY_DIR}/omniORB*_rt.dll")
      get_filename_component(libomniorb ${libomniorb} NAME_WE)

      file(GLOB libomnithread RELATIVE ${CHOREONOID_OMNIORB_BINARY_DIR} "${CHOREONOID_OMNIORB_BINARY_DIR}/omnithread*_rt.dll")
      get_filename_component(libomnithread ${libomnithread} NAME_WE)

      file(GLOB libomnidynamic RELATIVE ${CHOREONOID_OMNIORB_BINARY_DIR} "${CHOREONOID_OMNIORB_BINARY_DIR}/omniDynamic*_rt.dll")
      get_filename_component(libomnidynamic ${libomnidynamic} NAME_WE)

      set(CHOREONOID_OMNIORB_DLL_BASES ${libomniorb} ${libomnithread} ${libomnidynamic})

      if(INSTALL_RUNTIME_DEPENDENCIES)
	foreach(library ${CHOREONOID_OMNIORB_DLL_BASES})
	  install(PROGRAMS "${CHOREONOID_OMNIORB_BINARY_DIR}/${library}.dll" DESTINATION bin CONFIGURATIONS Release RelWithDebInfo MinSizeRel)
	  install(PROGRAMS "${CHOREONOID_OMNIORB_BINARY_DIR}/${library}d.dll" DESTINATION bin CONFIGURATIONS Debug)
	endforeach()
      endif()
    endif()
  endif()

  include_directories(${CHOREONOID_OMNIORB_INCLUDE_DIRS})
  link_directories(${CHOREONOID_OMNIORB_LIBRARY_DIRS})
  list(APPEND CHOREONOID_SDK_INCLUDE_DIRS ${CHOREONOID_OMNIORB_INCLUDE_DIRS})
  list(APPEND CHOREONOID_SDK_LIBRARY_DIRS ${CHOREONOID_OMNIORB_LIBRARY_DIRS})
  add_definitions(${CHOREONOID_OMNIORB_CFLAGS})

  set(CHOREONOID_OMNIORB_DIR ${CHOREONOID_OMNIORB_DIR} CACHE PATH "The top directory of omniORB")
  set(CHOREONOID_OMNIORB_CFLAGS ${CHOREONOID_OMNIORB_CFLAGS} CACHE STRING "Compile flags for omniORB")

  if(NOT CHOREONOID_OMNIORB_FOUND)
    message(FATAL_ERROR "CORBA-related modules require the omniORB library but the library is not found.")
  endif()

  if(ENABLE_PYTHON)
    install(DIRECTORY ${PROJECT_BINARY_DIR}/${CHOREONOID_PYTHON_SUBDIR} DESTINATION ${CHOREONOID_PLUGIN_SUBDIR}
      FILES_MATCHING PATTERN "*.py")
  endif()

endif(ENABLE_CORBA)

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
  set(USE_BUNDLED_LIBYAML_DEFAULT ON)
else()
  set(USE_BUNDLED_LIBYAML_DEFAULT OFF)
endif()
option(USE_BUNDLED_LIBYAML "Use the yaml library bundled with Choreonoid" ${USE_BUNDLED_LIBYAML_DEFAULT})
if(NOT USE_BUNDLED_LIBYAML)
  pkg_check_modules(LIBYAML yaml-0.1)
endif()
if((NOT LIBYAML_FOUND) OR USE_BUNDLED_LIBYAML)
  set(libyaml_dir thirdparty/libyaml-0.2.5)
  add_subdirectory(${libyaml_dir})
  set(LIBYAML_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/${libyaml_dir}/include)
  set(LIBYAML_LIBRARY_DIRS "")
  set(LIBYAML_LIBRARIES yaml)
endif()

# zlib
if(MSVC)
  include_directories(${PROJECT_SOURCE_DIR}/thirdparty/zlib-1.2.13)
  add_subdirectory(thirdparty/zlib-1.2.13)
endif()

# libzip
if(NOT MSVC)
  pkg_check_modules(libzip libzip)
  if(NOT libzip_FOUND)
    message(FATAL_ERROR "libzip is not found.")
  endif()
  include_directories(${libzip_INCLUDE_DIRS})
  link_directories(${libzip_LIBRARY_DIRS})
else()
  include_directories(${PROJECT_SOURCE_DIR}/thirdparty/libzip-1.9.2/lib)
  add_subdirectory(thirdparty/libzip-1.9.2)
  set(libzip_LIBRARIES zip)
endif()

# png
if(MSVC)
  set(USE_BUNDLED_LIBPNG_DEFAULT ON)
else()
  set(USE_BUNDLED_LIBPNG_DEFAULT OFF)
endif()
option(USE_BUNDLED_LIBPNG "Use the PNG library bunded with Choreonoid" ${USE_BUNDLED_LIBPNG_DEFAULT})
if(NOT USE_BUNDLED_LIBPNG)
  find_package(PNG)
endif()
if(PNG_FOUND AND NOT USE_BUNDLED_LIBPNG)
  list(APPEND CHOREONOID_SDK_INCLUDE_DIRS ${PNG_INCLUDE_DIRS})
else()
  set(PNG_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/thirdparty/lpng1232)
  set(PNG_LIBRARIES png zlib)
  add_subdirectory(thirdparty/lpng1232)
endif()

# jpeg
if(MSVC)
  set(USE_BUNDLED_LIBJPEG_DEFAULT ON)
else()
  set(USE_BUNDLED_LIBJPEG_DEFAULT OFF)
endif()
option(USE_BUNDLED_LIBJPEG "Use the JPEG library bunded with Choreonoid" ${USE_BUNDLED_LIBJPEG_DEFAULT})
if(NOT USE_BUNDLED_LIBJPEG)
  find_package(JPEG)
endif()
if(JPEG_FOUND AND NOT USE_BUNDLED_LIBJPEG)
  list(APPEND CHOREONOID_SDK_INCLUDE_DIRS ${JPEG_INCLUDE_DIR})
else()
  add_subdirectory(thirdparty/jpeg-9c)
  set(JPEG_LIBRARY jpeg)
  set(JPEG_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/thirdparty/jpeg-9c")
  set(JPEG_LIBRARIES jpeg)
endif()

# fmtlib
if(MSVC)
  set(USE_BUNDLED_FMTLIB_DEFAULT ON)
else()
  set(USE_BUNDLED_FMTLIB_DEFAULT OFF)
endif()
option(USE_BUNDLED_FMTLIB "Use the fmt library bunded with Choreonoid" ${USE_BUNDLED_FMTLIB_DEFAULT})
if(NOT USE_BUNDLED_FMTLIB)
  find_package(fmt QUIET)
endif()
if((NOT fmt_FOUND) OR USE_BUNDLED_FMTLIB)
  add_subdirectory(thirdparty/fmt-7.1.3)
  if(INSTALL_SDK)
    install(DIRECTORY ${PROJECT_SOURCE_DIR}/thirdparty/fmt-7.1.3/include/fmt DESTINATION ${CHOREONOID_HEADER_SUBDIR})
    if(NOT CMAKE_VERSION VERSION_LESS 3.13.0)
      install(TARGETS fmt LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)
    elseif(UNIX)
      install(FILES ${PROJECT_BINARY_DIR}/lib/libfmt.a DESTINATION lib)
    endif()
  endif()
endif()

# uuid
if(UNIX)
  pkg_check_modules(LIBUUID uuid REQUIRED)
endif()

# pugixml
set(USE_BUNDLED_PUGIXML_DEFAULT ON)
option(USE_BUNDLED_PUGIXML "Use the pugixml library bundled with Choreonoid" ${USE_BUNDLED_PUGIXML_DEFAULT})
if(NOT USE_BUNDLED_PUGIXML)
  pkg_check_modules(PUGIXML pugixml>=1.11 QUIET)
endif()
if(NOT PUGIXML_STATIC_FOUND OR USE_BUNDLED_PUGIXML)
  add_subdirectory(thirdparty/pugixml-1.11)
  set(PUGIXML_STATIC_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/thirdparty/pugixml-1.11)
  set(PUGIXML_STATIC_LIBRARIES pugixml)
  unset(PUGIXML_STATIC_LIBRARY_DIRS)
endif()

add_subdirectory(src)
add_subdirectory(include)

option(ENABLE_SAMPLES "Enable samples in the sample directory" ON)

add_subdirectory(share)

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

    install(PROGRAMS ${CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS_RELEASE} DESTINATION bin)
  endif()
endif()

if(INSTALL_SDK)
  add_subdirectory(cmake)
endif()
add_subdirectory(misc)
