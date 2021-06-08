find_package(Doxygen)

set(target_dirs
  Util
  AISTCollisionDetector
  Body
  Base
  BodyPlugin
  ManipulatorPlugin
  MediaPlugin
  PythonPlugin
  PythonSimScriptPlugin
  )
  
unset(input_files)
set(DOXYGEN_INPUT_DIRECTORIES "")
foreach(directory ${target_dirs})
  set(abs_directory ${PROJECT_SOURCE_DIR}/src/${directory})
  set(DOXYGEN_INPUT_DIRECTORIES "${DOXYGEN_INPUT_DIRECTORIES} ${abs_directory}")
  file(GLOB files ${abs_directory}/*.h ${abs_directory}/*.cpp ${abs_directory}/*.c)
  list(APPEND input_files ${files})
endforeach()

configure_file(Doxyfile.in Doxyfile @ONLY)

set(output_dir ${CMAKE_CURRENT_BINARY_DIR}/html)
file(MAKE_DIRECTORY ${output_dir})

add_custom_command(
  OUTPUT ${output_dir}/index.html
  COMMAND doxygen
  DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile ${input_files}
)

add_custom_target(doc ALL DEPENDS ${output_dir}/index.html)

set_property(
  TARGET doc
  APPEND PROPERTY ADDITIONAL_CLEAN_FILES ${output_dir})

install(DIRECTORY ${output_dir} DESTINATION ${CNOID_DOC_SUBDIR})
