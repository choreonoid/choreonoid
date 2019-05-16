# @author Shin'ichiro Nakaoka

if(APPLE)
  # Gstreamer doesn't support MacOS X
  #return()
endif()

option(BUILD_MEDIA_PLUGIN "Building MediaPlugin" OFF)

if(NOT BUILD_MEDIA_PLUGIN)
  return()
endif()

#set(CMAKE_BUILD_TYPE Debug)

if(UNIX)

  set(sources
    MediaPlugin.cpp
    MediaItem.cpp
    GSMediaView.cpp
    AudioItem.cpp
    )

  pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0 gstreamer-video-1.0 gstreamer-base-1.0)
  include_directories(${GSTREAMER_INCLUDE_DIRS})

  pkg_check_modules(SNDFILE REQUIRED sndfile)
  include_directories(${SNDFILE_INCLUDE_DIRS})
  set_source_files_properties(AudioItem.cpp PROPERTIES COMPILE_DEFINITIONS "CNOID_MEDIA_PLUGIN_USE_LIBSNDFILE")

  set(libraries 
    CnoidUtil CnoidBase 
    ${GSTREAMER_LIBRARIES}
    ${SNDFILE_LIBRARIES}
    X11
    )

  if(NOT APPLE)
    pkg_check_modules(LIBPULSE REQUIRED libpulse)
    include_directories(${LIBPULSE_INCLUDE_DIRS})
    set(sources ${sources} PulseAudioManager.cpp)
    set_source_files_properties(MediaPlugin.cpp PROPERTIES COMPILE_DEFINITIONS "CNOID_MEDIA_PLUGIN_USE_PULSEAUDIO")
    set(libraries ${libraries} ${LIBPULSE_LIBRARIES})
  endif()

elseif(WIN32)
  set(sources
    MediaPlugin.cpp
    MediaItem.cpp
    DSMediaView.cpp
    AudioItem.cpp
    )
  set(libraries CnoidUtil CnoidBase Strmiids.lib)
endif()

set(headers
  MediaUtil.h
  MediaItem.h
  AudioItem.h
  )

set(target CnoidMediaPlugin)
make_gettext_mofiles(${target} mofiles)
add_cnoid_plugin(${target} SHARED ${sources} ${headers} ${mofiles})
target_link_libraries(${target} ${libraries})
apply_common_setting_for_plugin(${target} "${headers}")

message(STATUS "libraries = ${libraries}")

if(ENABLE_PYTHON)
  add_subdirectory(pybind11)
endif()
