find_package(Doxygen)

set(target_dirs
  Util
  Base
  Body
  AISTCollisionDetector
  BodyPlugin
  PoseSeqPlugin
  Python
  PythonPlugin
  PythonSimScriptPlugin
  Corba
  CorbaPlugin
  OpenRTM
  OpenRTMPlugin 
  )
  
unset(input_files)
set(DOXYGEN_INPUT_DIRECTORIES "")
foreach(directory ${target_dirs})
  set(abs_directory ${PROJECT_SOURCE_DIR}/src/${directory})
  set(DOXYGEN_INPUT_DIRECTORIES "${DOXYGEN_INPUT_DIRECTORIES} ${abs_directory}")
  file(GLOB_RECURSE files ${abs_directory}/*.h ${abs_directory}/*.cpp ${abs_directory}/*.c)
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

install(DIRECTORY ${output_dir} DESTINATION ${CNOID_DOC_SUBDIR})
