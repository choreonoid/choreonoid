#include "PyUtil.h"
#include "../SceneLoader.h"
#include "../StdSceneLoader.h"
#include "../StdSceneWriter.h"
#include <iostream>

namespace py = pybind11;
using namespace cnoid;

namespace cnoid {

void exportPySceneLoader(py::module& m)
{
    py::class_<SceneLoader>(m, "SceneLoader")
        .def(py::init())
    .def_static("availableFileExtensions", []() { return SceneLoader::availableFileExtensions(); })
    .def("setDefaultDivisionNumber", &SceneLoader::setDefaultDivisionNumber)
    .def("setDefaultCreaseAngle", &SceneLoader::setDefaultCreaseAngle)
    .def("load", [](SceneLoader &self, const std::string &filename) {
        bool res; SgNodePtr p = self.load(filename, res);
        if (!res) std::cerr << "unsupported format : " << filename << std::endl;
        return p; })
    //
    .def("setMessageSinkStdErr", [](SceneLoader &self) {
        self.setMessageSink(std::cerr); })
    ;

    py::class_<StdSceneLoader>(m, "StdSceneLoader")
    .def(py::init())
    .def("setDefaultDivisionNumber", &StdSceneLoader::setDefaultDivisionNumber)
    .def("setDefaultCreaseAngle", &StdSceneLoader::setDefaultCreaseAngle)
    .def("load", [](StdSceneLoader &self, const std::string &filename) {
        SgNodePtr p = self.load(filename); return p; })
    //
    .def("setMessageSinkStdErr", [](StdSceneLoader &self) {
        self.setMessageSink(std::cerr); })
    ;

    py::class_<StdSceneWriter>(m, "StdSceneWriter")
    .def(py::init())
    .def_property("ExtModelFileMode",
                  &StdSceneWriter::extModelFileMode,
                  &StdSceneWriter::setExtModelFileMode)
    .def_property("TopGroupNodeSkippingEnabled",
                  &StdSceneWriter::isTopGroupNodeSkippingEnabled,
                  &StdSceneWriter::setTopGroupNodeSkippingEnabled)
    .def_property("TransformIntegrationEnabled",
                  &StdSceneWriter::isTransformIntegrationEnabled,
                  &StdSceneWriter::setTransformIntegrationEnabled)
    .def_property("AppearanceEnabled",
                  &StdSceneWriter::isAppearanceEnabled,
                  &StdSceneWriter::setAppearanceEnabled)
    .def_property("MeshEnabled",
                  &StdSceneWriter::isMeshEnabled,
                  &StdSceneWriter::setMeshEnabled)
    .def("writeScene",
         [](StdSceneWriter &self, const std::string &filename, SgNode *node) {
             return self.writeScene(filename, node); })
    .def("setMessageSinkStdErr", [](StdSceneWriter &self) {
        self.setMessageSink(std::cerr); })
    ;
}

}
