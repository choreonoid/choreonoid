if(NOT BUILD_POSE_SEQ_PLUGIN)
  return()
endif()

set(motion_files
  motion/GR001/SampleMotion1.pseq
  motion/RIC30/SampleMotion1.pseq
  motion/SR1/SR1WalkPattern2.pseq
  )

foreach(file ${motion_files})
  get_filename_component(dir ${file} DIRECTORY)
  get_filename_component(name ${file} NAME)
  file(COPY ${file} DESTINATION ${CNOID_BINARY_SHARE_DIR}/${dir})
  install(FILES ${file} DESTINATION ${CNOID_SHARE_SUBDIR}/${dir})
endforeach()

install_project_files(GR001.cnoid RIC30.cnoid)
