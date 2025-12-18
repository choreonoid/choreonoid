option(BUILD_MEDIA_PLUGIN "Building MediaPlugin" OFF)

if(NOT BUILD_MEDIA_PLUGIN)
  return()
endif()

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

  pkg_check_modules(LIBPULSE REQUIRED libpulse)
  include_directories(${LIBPULSE_INCLUDE_DIRS})
  set(sources ${sources} PulseAudioManager.cpp)
  set_source_files_properties(MediaPlugin.cpp PROPERTIES COMPILE_DEFINITIONS "CNOID_MEDIA_PLUGIN_USE_PULSEAUDIO")
  
  set(libraries PRIVATE ${GSTREAMER_LIBRARIES} ${SNDFILE_LIBRARIES} ${LIBPULSE_LIBRARIES} X11)

elseif(WIN32)
  option(USE_MEDIA_FOUNDATION "Use Media Foundation instead of DirectShow for MediaView" ON)
  mark_as_advanced(USE_MEDIA_FOUNDATION)

  set(sources
    MediaPlugin.cpp
    MediaItem.cpp
    AudioItem.cpp
    )

  if(USE_MEDIA_FOUNDATION)
    set(sources ${sources} MFMediaView.cpp)
    set(libraries PRIVATE Mf.lib Mfplat.lib Mfuuid.lib Mfreadwrite.lib Strmiids.lib)
    set_source_files_properties(MediaPlugin.cpp PROPERTIES COMPILE_DEFINITIONS "USE_MEDIA_FOUNDATION")
  else()
    set(sources ${sources} DSMediaView.cpp)
    set(libraries PRIVATE Strmiids.lib quartz.lib)
  endif()
endif()

set(headers
  MediaUtil.h
  MediaItem.h
  AudioItem.h
  )

set(target CnoidMediaPlugin)
choreonoid_make_gettext_mo_files(${target} mofiles)
choreonoid_add_plugin(${target} ${sources} ${mofiles} HEADERS ${headers})
target_link_libraries(${target} ${libraries})

if(ENABLE_PYTHON)
  add_subdirectory(pybind11)
endif()
