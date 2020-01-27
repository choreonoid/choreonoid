include_directories(include win32)
add_library(yaml STATIC
  src/api.c
  src/dumper.c
  src/emitter.c
  src/loader.c
  src/parser.c
  src/reader.c
  src/scanner.c
  src/writer.c
  src/yaml_private.h
  include/yaml.h
  win32/config.h
)
set_target_properties(yaml PROPERTIES COMPILE_DEFINITIONS "HAVE_CONFIG_H;YAML_DECLARE_STATIC")
if(MSVC)
  set_target_properties(yaml PROPERTIES DEBUG_POSTFIX d)
else()
  set_target_properties(yaml PROPERTIES COMPILE_OPTIONS -fPIC)
endif()
