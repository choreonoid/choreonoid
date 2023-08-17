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

    py::class_<AssimpSceneLoader>(m, "AssimpSceneLoader", R"__CNOID__(
AssimpSceneLoader : loading mesh file as a scene for Util/SceneGraph in choreonoid using Assimp ( https://github.com/assimp/assimp )
)__CNOID__")
        .def(py::init<>())
        .def("load", &AssimpSceneLoader::load, R"__CNOID__(
Loading a mesh file using Assimp ( https://github.com/assimp/assimp )

Args:
    filename (str) : Filename to be loaded
Returns:
    cnoid.Util.SgGroup : Root object of loaded mesh
)__CNOID__")
        .def("setMessageSinkStdErr", &AssimpSceneLoader::setMessageSinkStdErr, R"__CNOID__(
Setting MessageSkin to stderr. It is required to read error messages in console

Args:
    None
)__CNOID__")
        ;

    py::class_<AssimpSceneWriter>(m, "AssimpSceneWriter", R"__CNOID__(
AssimpSceneWriter : Saving a scene for Util/SceneGraph in choreonoid as a mesh file using Assimp ( https://github.com/assimp/assimp )
)__CNOID__")
        .def(py::init<>())
        .def("writeScene", &AssimpSceneWriter::writeScene, R"__CNOID__(
Writing scene as a mesh file using Assimp ( https://github.com/assimp/assimp )

Args:
    filename (str) : Filename to be saved
    node (cnoid.Util.SgNode) : An object of SceneGraph to be saved
Returns:
    boolean : If the process succeeds, True is returned.

)__CNOID__")
        .def("setVerbose", &AssimpSceneWriter::setVerbose, R"__CNOID__(
Setting debug message level

Args:
    on (boolean) : Set debug message level
)__CNOID__")
        .def("generatePrimitiveMesh", &AssimpSceneWriter::generatePrimitiveMesh, R"__CNOID__(
Setting for generating meshes for primitives

Args:
    on (boolean) : If True, this scene writer generates a mesh file including meshes converted from primitive type
)__CNOID__")
        .def("setMessageSinkStdErr", &AssimpSceneWriter::setMessageSinkStdErr, R"__CNOID__(
Setting MessageSkin to stderr. It is required to read error messages in console

Args:
    None
)__CNOID__")
        .def_property("outputType", &AssimpSceneWriter::getOutputType, &AssimpSceneWriter::setOutputType,
                  R"__CNOID__(
Setting output type of saving mesh. If not set, output type is determined from filename.
For saving mesh as 'stl', you shoud set outputType = 'stlb', if you want a stl binary file.

Args:
    output_type (str) : output type to be set

Returns
    str : Current output type
)__CNOID__")
        ;

}
