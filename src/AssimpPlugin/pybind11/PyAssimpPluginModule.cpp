/*!
  @author Yohei Kakiuchi
*/

#include <cnoid/PyUtil>
#include <cnoid/PyEigenTypes>

#include <cnoid/SceneGraph>
#include <cnoid/AssimpSceneLoader>
#include <cnoid/AssimpSceneWriter>

using namespace cnoid;
namespace py = pybind11;

PYBIND11_MODULE(AssimpPlugin, m)
{
    m.doc() = "Choreonoid Assimp Utility module";

    py::module::import("cnoid.Util");

    m.def("registerAssimpSceneLoader", []() { AssimpSceneLoader::initializeClass(); });

    py::class_<AssimpSceneLoader>(m, "AssimpSceneLoader")
        .def(py::init<>())
        .def("load", &AssimpSceneLoader::load)
        .def("setMessageSinkStdErr", &AssimpSceneLoader::setMessageSinkStdErr)
        ;

    py::class_<AssimpSceneWriter>(m, "AssimpSceneWriter")
        .def(py::init<>())
        .def("writeScene", &AssimpSceneWriter::writeScene)
        .def("setVerbose", &AssimpSceneWriter::setVerbose)
        .def("generatePrimitiveMesh", &AssimpSceneWriter::generatePrimitiveMesh)
        .def("setMessageSinkStdErr", &AssimpSceneWriter::setMessageSinkStdErr)
        .def_property("outputType", &AssimpSceneWriter::getOutputType, &AssimpSceneWriter::setOutputType)
        ;

}


