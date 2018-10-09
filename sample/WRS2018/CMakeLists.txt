if(NOT ENABLE_GUI)
  return()
endif()

option(BUILD_WRS2018_SAMPLES "Building WRS2018 samples" ON)
if(NOT BUILD_WRS2018_SAMPLES)
  return()
endif()

install(FILES
  T1.cnoid
  T2.cnoid
  T3.cnoid
  T4.cnoid
  T5.cnoid
  T6.cnoid
  DoubleArmV7S.cnoid
  DoubleArmV7A.cnoid
  AizuSpiderSS.cnoid
  AizuSpiderSA.cnoid
  WAREC1.cnoid
  AISTSimulator.cnoid
  AGXSimulator.cnoid
  WRSUtil.py
  T1-AizuSpiderSA.py
  T1-AizuSpiderSS.py  
  T1-DoubleArmV7A.py
  T1-DoubleArmV7S.py
  T2-DoubleArmV7S.py
  T2-DoubleArmV7A.py
  T2-AizuSpiderSS.py 
  T2-AizuSpiderSA.py
  T3-DoubleArmV7S.py
  T3-DoubleArmV7A.py
  T3-AizuSpiderSS.py 
  T3-AizuSpiderSA.py
  T4-DoubleArmV7S.py
  T4-DoubleArmV7A.py
  T4-AizuSpiderSS.py
  T4-AizuSpiderSA.py
  T5-DoubleArmV7S.py
  T5-DoubleArmV7A.py
  T5-AizuSpiderSA.py
  T5-AizuSpiderSS.py
  T6-DoubleArmV7A.py
  DESTINATION ${CNOID_SHARE_SUBDIR}/project/WRS2018)

add_subdirectory(src)
