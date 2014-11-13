
# @author Shin'ichiro Nakaoka

set(PYTHON_SUFFIX -python2.7)

add_cnoid_python_module(CnoidPyQtCore
  PyQtCore.cpp
  )

target_link_libraries(CnoidPyQtCore
  ${QT_LIBRARIES}
  ${PYTHON_LIBRARIES}
  ${Boost_PYTHON_LIBRARY}
  )

add_cnoid_python_module(CnoidPyQtGui
  PyQtGui.cpp
  )

target_link_libraries(CnoidPyQtGui
  ${QT_LIBRARIES}
  ${PYTHON_LIBRARIES}
  ${Boost_PYTHON_LIBRARY}
  )


add_cnoid_python_module(CnoidPyBase
  PyBase.cpp
  PyQtEx.cpp
  PyItems.cpp
  PyToolBars.cpp
  PyViews.cpp
)

target_link_libraries(CnoidPyBase
  CnoidBase 
  ${PYTHON_LIBRARIES}
  ${Boost_PYTHON_LIBRARY}
  )
