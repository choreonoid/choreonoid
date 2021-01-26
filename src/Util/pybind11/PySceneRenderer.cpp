#include "PyUtil.h"
#include "../SceneRenderer.h"
#include "../SceneCameras.h"
#include "../SceneLights.h"
#include "../SceneEffects.h"

using namespace cnoid;
namespace py = pybind11;

namespace cnoid {

void exportPySceneRenderer(py::module& m)
{
    py::class_<SceneRenderer>(m, "SceneRenderer")
        .def_property_readonly("name", &SceneRenderer::name)
        .def_property_readonly("sceneRoot", &SceneRenderer::sceneRoot)
        .def_property_readonly("scene", &SceneRenderer::scene)
        .def_property_readonly("clearScene", &SceneRenderer::clearScene)
        .def_property_readonly("numCameras", &SceneRenderer::numCameras)
        .def("getCamera", &SceneRenderer::camera)
        .def("getCameraPath", &SceneRenderer::cameraPath)
        .def_property_readonly(
            "currentCameraPosition",
            [](SceneRenderer& self) -> Isometry3::MatrixType { return self.currentCameraPosition().matrix(); })
        .def_property_readonly("sigCamerasChanged", &SceneRenderer::sigCamerasChanged)
        .def_property_readonly("currentCamera", &SceneRenderer::currentCamera)
        .def_property_readonly("currentCameraIndex", &SceneRenderer::currentCameraIndex)
        .def("setCurrentCamera", (void(SceneRenderer::*)(int)) &SceneRenderer::setCurrentCamera)
        .def("setCurrentCamera", (bool(SceneRenderer::*)(SgCamera*)) &SceneRenderer::setCurrentCamera)
        .def_property_readonly("sigCurrentCameraChanged", &SceneRenderer::sigCurrentCameraChanged)
        .def("getSimplifiedCameraPathStrings", &SceneRenderer::simplifiedCameraPathStrings)
        .def("findCameraPath", &SceneRenderer::findCameraPath)
        .def("setCurrentCameraPath", &SceneRenderer::setCurrentCameraPath)
        .def("setCurrentCameraAutoRestorationMode", &SceneRenderer::setCurrentCameraAutoRestorationMode)
        .def_property_readonly("numLights", &SceneRenderer::numLights)
        .def("getLight",
             [](SceneRenderer& self, int index){
                 SgLight* light = nullptr;
                 if(index < self.numLights()){
                     Isometry3 T;
                     self.getLightInfo(index, light, T);
                 }
                 return light;
             })
        .def("setAsDefaultLight", &SceneRenderer::setAsDefaultLight)
        .def("unsetDefaultLight", &SceneRenderer::unsetDefaultLight)
        .def_property("headLight", &SceneRenderer::headLight, &SceneRenderer::setHeadLight)
        .def("setHeadLight", &SceneRenderer::setHeadLight)
        .def("enableAdditionalLights", &SceneRenderer::enableAdditionalLights)
        .def("enableFog", &SceneRenderer::enableFog)
        .def("isFogEnabled", &SceneRenderer::isFogEnabled)
        .def_property_readonly("numFogs", &SceneRenderer::numFogs)
        .def("getFog", &SceneRenderer::fog)
        .def("extractPreprocessedNodes", &SceneRenderer::extractPreprocessedNodes)
        .def("setProperty",
             [](SceneRenderer& self, const std::string& key, bool value){
                 self.setProperty(key, value);
             })
        .def("setProperty",
             [](SceneRenderer& self, const std::string& key, int value){
                 self.setProperty(key, value);
             })
        .def("setProperty",
             [](SceneRenderer& self, const std::string& key, double value){
                 self.setProperty(key, value);
             })
        .def("getProperty",
             [](SceneRenderer& self, const std::string& key, bool defaultValue){
                 return self.property(key, defaultValue);
             })
        .def("getProperty",
             [](SceneRenderer& self, const std::string& key, int defaultValue){
                 return self.property(key, defaultValue);
             })
        .def("getProperty",
             [](SceneRenderer& self, const std::string& key, double defaultValue){
                 return self.property(key, defaultValue);
             })
        ;
}

}

