file(COPY src/xacro DESTINATION ${PROJECT_BINARY_DIR}/${CHOREONOID_PYTHON_SUBDIR}/cnoid)
configure_file(scripts/cnoid-xacro ${PROJECT_BINARY_DIR}/bin/cnoid-xacro)

install(DIRECTORY src/xacro DESTINATION ${CHOREONOID_PYTHON_SUBDIR}/cnoid)
install(FILES scripts/cnoid-xacro DESTINATION bin)
