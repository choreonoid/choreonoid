
# @author Shin'ichiro Nakaoka

if(BUILD_OPENHRP_PLUGIN_FOR_3_0)
  idl_compile_python(OpenHRP30PythonStubs OpenHRP/3.0 OpenHRP_3_0 OpenHRP
    OpenHRPCommon
    DynamicsSimulator
    ModelLoader
    CollisionDetector
    Controller
    OnlineViewer
    ViewSimulator
    DEPENDS CnoidOpenHRP3.0Plugin
    )
endif()

