# @author Shin'ichiro Nakaoka

if(NOT ENABLE_GUI)
  return()
endif()

option(BUILD_OPENHRP_SAMPLES "Building OpenHRP samples" OFF)

if(NOT BUILD_OPENHRP_SAMPLES)
  return()
elseif(NOT BUILD_OPENHRP_PLUGIN)
  message(FATAL_ERROR "OpenHRP samples need to enable BUILD_OPENHRP_PLUGIN.")
endif()

if(BUILD_OPENHRP_PLUGIN_FOR_3_0)
  set(versions 3.1 3.0)
else()
  set(versions 3.1)
endif()

foreach(version ${versions})

  if(version EQUAL 3.0)
    set(defver "OPENHRP_3_0")
  else()
    set(defver "OPENHRP_3_1")
  endif()

  # SR1 walk
  set(target openhrp${version}-sr1-walk-controller)
  add_cnoid_executable(${target} SimpleControllerWrapper.cpp ../SimpleController/SR1WalkPatternController.cpp)
  set_target_properties(${target} PROPERTIES COMPILE_DEFINITIONS "${defver};MODELFILE=\"SR1/SR1.body\"")
  target_link_libraries(${target} CnoidOpenHRP${version} CnoidBody)
  
  # SR1 lifting up
  set(target openhrp${version}-sr1-liftup-controller)
  add_cnoid_executable(${target} SimpleControllerWrapper.cpp ../SimpleController/SR1LiftupController.cpp)
  set_target_properties(${target} PROPERTIES COMPILE_DEFINITIONS "${defver};MODELFILE=\"SR1/SR1.body\"")
  target_link_libraries(${target} CnoidOpenHRP${version} CnoidBody)
  
  # PA10 Pickup
  set(target openhrp${version}-pa10-pickup-controller)
  add_cnoid_executable(${target} SimpleControllerWrapper.cpp ../SimpleController/PA10PickupController.cpp)
  set_target_properties(${target} PROPERTIES COMPILE_DEFINITIONS "${defver};MODELFILE=\"PA10/PA10.body\"")
  target_link_libraries(${target} CnoidOpenHRP${version} CnoidBody)
  
  # test program
  set(target openhrp${version}-online-viewer-test)
  add_cnoid_executable(${target} olv-test.cpp)
  set_target_properties(${target} PROPERTIES COMPILE_DEFINITIONS ${defver})
  target_link_libraries(${target} CnoidOpenHRP${version} CnoidBody ${OMNIDYNAMIC_LIBRARIES})
  
  # Openhrp modelLoader debug program
  #set(target test-model-loader)
  #add_cnoid_executable(${target} test-model-loader.cpp)
  #set_target_properties(${target} PROPERTIES COMPILE_DEFINITIONS ${defver})
  #target_link_libraries(${target} CnoidBody CnoidOpenHRP${version} ${OMNIDYNAMIC_LIBRARIES})
  
endforeach()

install(FILES 
  OpenHRP-SR1Walk.cnoid
  OpenHRP-SR1Liftup.cnoid
  OpenHRP-PA10Pickup.cnoid
  OpenHRP-OnlineViewer.cnoid
  DESTINATION ${CNOID_SHARE_SUBDIR}/project)
