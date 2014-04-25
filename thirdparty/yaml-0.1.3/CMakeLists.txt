
if(NOT USE_EXTERNAL_YAML)

  set(sources
    src/api.c
    src/dumper.c
    src/emitter.c
    src/loader.c
    src/parser.c
    src/reader.c
    src/scanner.c
    src/writer.c
  )

  include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
  include_directories(${CMAKE_CURRENT_SOURCE_DIR}/win32)

  add_library(yaml STATIC ${sources})

  set_target_properties(yaml PROPERTIES
    COMPILE_DEFINITIONS "HAVE_CONFIG_H;YAML_DECLARE_STATIC;"
    DEBUG_POSTFIX d
    )

endif()
