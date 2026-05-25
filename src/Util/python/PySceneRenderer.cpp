#include "PyUtil.h"
#include "../SceneRenderer.h"
#include "../SceneCameras.h"
#include "../SceneLights.h"
#include "../SceneEffects.h"
#include <nanobind/eigen/dense.h>
#include <nanobind/stl/vector.h>

using namespace cnoid;
namespace nb = nanobind;

namespace cnoid {

void exportPySceneRenderer(nb::module_& m)
{
    nb::class_<SceneRenderer>(m, "SceneRenderer")
        .def_prop_ro("name", &SceneRenderer::name)
        .def_prop_ro("sceneRoot", &SceneRenderer::sceneRoot)
        .def_prop_ro("scene", &SceneRenderer::scene)
        .def("clearScene", &SceneRenderer::clearScene)
        .def_prop_ro("numCameras", &SceneRenderer::numCameras)
        .def("getCamera", &SceneRenderer::camera)
        .def("getCameraPath", &SceneRenderer::cameraPath)
        .def_prop_ro(
            "currentCameraPosition",
            [](SceneRenderer& self) -> Isometry3::MatrixType { return self.currentCameraPosition().matrix(); })
        .def_prop_ro("sigCameraListChanged", &SceneRenderer::sigCameraListChanged)
        .def_prop_ro("currentCamera", &SceneRenderer::currentCamera)
        .def_prop_ro("currentCameraIndex", &SceneRenderer::currentCameraIndex)
        .def("setCurrentCamera", (void(SceneRenderer::*)(int)) &SceneRenderer::setCurrentCamera)
        .def("setCurrentCamera", (bool(SceneRenderer::*)(SgCamera*)) &SceneRenderer::setCurrentCamera)
        .def_prop_ro("sigCurrentCameraSelectionChanged", &SceneRenderer::sigCurrentCameraSelectionChanged)
        .def("getSimplifiedCameraPathStrings", &SceneRenderer::simplifiedCameraPathStrings)
        .def("findCameraPath", &SceneRenderer::findCameraPath)
        .def("setCurrentCameraPath", &SceneRenderer::setCurrentCameraPath)
        .def("setCurrentCameraAutoRestorationMode", &SceneRenderer::setCurrentCameraAutoRestorationMode)
        .def_prop_ro("numAdditionalLights", &SceneRenderer::numAdditionalLights)
        .def("getLight",
             [](SceneRenderer& self, int index){
                 SgLight* light = nullptr;
                 if(index < self.numAdditionalLights()){
                     Isometry3 T;
                     self.getLightInfo(index, light, T);
                 }
                 return light;
             })
        .def_prop_rw("headLight", &SceneRenderer::headLight, &SceneRenderer::setHeadLight)
        .def("setHeadLight", &SceneRenderer::setHeadLight)
        .def_prop_rw("worldLight", &SceneRenderer::worldLight, &SceneRenderer::setWorldLight)
        .def("setWorldLight", &SceneRenderer::setWorldLight)
        .def_prop_ro("worldLightTransform", &SceneRenderer::worldLightTransform, nb::rv_policy::reference)
        .def("enableAdditionalLights", &SceneRenderer::enableAdditionalLights)
        .def("enableFog", &SceneRenderer::enableFog)
        .def("isFogEnabled", &SceneRenderer::isFogEnabled)
        .def_prop_ro("numFogs", &SceneRenderer::numFogs)
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
