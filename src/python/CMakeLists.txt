
configure_file(numpy3d.py ${PROJECT_BINARY_DIR}/${CNOID_PYTHON_SUBDIR} COPYONLY)
install(FILES numpy3d.py DESTINATION ${CNOID_PYTHON_SUBDIR}
  CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel)
