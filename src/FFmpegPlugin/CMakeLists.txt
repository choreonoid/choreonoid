option(BUILD_FFMPEG_PLUGIN "Building FFmpegPlugin" OFF)
mark_as_advanced(BUILD_FFMPEG_PLUGIN)

if(NOT BUILD_FFMPEG_PLUGIN)
  return()
endif()

if(MSVC)
  set(FFMPEG_DIR "" CACHE PATH "The top directory of the FFmpeg libraries")
  if(NOT FFMPEG_DIR)
    message(FATAL_ERROR "FFMPEG_DIR must be specified to build FFmpegPlugin")
  endif()
  set(FFMPEG_INCLUDE_DIRS "${FFMPEG_DIR}/include")
  set(FFMPEG_LIBRARY_DIRS "${FFMPEG_DIR}/lib")
  set(FFMPEG_LIBRARIES avutil avcodec avformat)
  install_runtime_dlls(${FFMPEG_DIR}/bin avcodec-58 avformat-58 avutil-56 swresample-3 libopenh264)
else()
  # The required packages can be installed by the following command in Ubuntu.
  # sudo apt install libavcodec-dev libavformat-dev libavutil-dev
  pkg_check_modules(LIBAVCODEC libavcodec)
  pkg_check_modules(LIBAVFORMAT libavformat)
  pkg_check_modules(LIBAVUTIL libavutil)
  set(FFMPEG_INCLUDE_DIRS ${LIBAVCODEC_INCLUDE_DIRS} ${LIBAVFORMAT_INCLUDE_DIRS} ${LIBAVUTIL_INCLUDE_DIRS})
  set(FFMPEG_LIBRARY_DIRS ${LIBAVCODEC_LIBRARY_DIRS} ${LIBAVFORMAT_LIBRARY_DIRS} ${LIBAVUTIL_LIBRARY_DIRS})
  set(FFMPEG_LIBRARIES ${LIBAVCODEC_LIBRARIES} ${LIBAVFORMAT_LIBRARIES} ${LIBAVUTIL_LIBRARIES})
endif()

include_directories(${FFMPEG_INCLUDE_DIRS})
link_directories(${FFMPEG_LIBRARY_DIRS})

set(target CnoidFFmpegPlugin)

set(sources
  FFmpegPlugin.cpp
  FFmpegMovieRecorderEncoder.h
  FFmpegMovieRecorderEncoder.cpp
  )

set(headers
  FFmpegMovieRecorderEncoder.h
  )

choreonoid_make_gettext_mo_files(${target} mofiles)
choreonoid_add_plugin(${target} ${sources} ${mofiles} HEADERS ${headers})
target_link_libraries(${target} PRIVATE ${FFMPEG_LIBRARIES})
