if(NOT ENABLE_GUI)
  return()
endif()

option(BUILD_WRS2018_SAMPLES "Building WRS2018 samples" ON)
if(NOT BUILD_WRS2018_SAMPLES)
  return()
endif()

set(project_files
  T1-T4-DoubleArmV7A.cnoid
  T1-T4-AizuSpiderSA.cnoid
  T2-DoubleArmV7A.cnoid
  T2-AizuSpiderSA.cnoid
  T3-DoubleArmV7A.cnoid
  T5-AizuSpiderSA.cnoid
  T5-DoubleArmV7A.cnoid
  )

install(FILES ${project_files} DESTINATION ${CNOID_SHARE_SUBDIR}/project/WRS2018)
