
# @author Shin'ichiro Nakaoka

set(PYTHON_SUFFIX -python2.7)

find_package(Shiboken REQUIRED)
find_package(PySide REQUIRED)

include_directories(
  ${SHIBOKEN_INCLUDE_DIR} 
  ${PYSIDE_INCLUDE_DIR} ${PYSIDE_INCLUDE_DIR}/QtCore ${PYSIDE_INCLUDE_DIR}/QtGui)

set(target CnoidPyBase)
add_cnoid_python_module(${target} PyBase.cpp)
target_link_libraries(${target}
  CnoidBase 
  ${PYTHON_LIBRARIES} ${Boost_PYTHON_LIBRARY}
  ${SHIBOKEN_LIBRARY} ${PYSIDE_LIBRARY}
  )
