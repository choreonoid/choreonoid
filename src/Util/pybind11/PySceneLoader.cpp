#include "PyUtil.h"
#include "../SceneLoader.h"
#include <iostream>

namespace py = pybind11;
using namespace cnoid;

namespace cnoid {

void exportPySceneLoader(py::module& m)
{
    py::class_<SceneLoader>(m, "SceneLoader")
        .def(py::init())
    .def("setDefaultDivisionNumber", &SceneLoader::setDefaultDivisionNumber)
    .def("setDefaultCreaseAngle", &SceneLoader::setDefaultCreaseAngle)
    .def("load", [](SceneLoader &self, const std::string &filename) {
        bool res;
        SgNodePtr p = self.load(filename, res);
        return p; })
    //
    .def("setMessageSinkStdErr", [](SceneLoader &self) {
        self.setMessageSink(std::cerr); })
    ;
}

}
