#include "PyReferenced.h"
#include "PyEigenTypes.h"
#include "../SceneDrawables.h"
#include "../CloneMap.h"

using namespace cnoid;
namespace py = pybind11;

namespace cnoid {

void exportPySceneDrawables(py::module& m)
{
    py::class_<SgMaterial, SgMaterialPtr, SgObject>(m, "SgMaterial")
        .def(py::init<>())
        .def_property("ambientIntensity", &SgMaterial::ambientIntensity, &SgMaterial::setAmbientIntensity)
        .def("setAmbientIntensity", &SgMaterial::setAmbientIntensity)
        .def_property("diffuseColor",
                      &SgMaterial::diffuseColor,
                      [](SgMaterial& self, const Vector3f c){ self.setDiffuseColor(c); })
        .def("setDiffuseColor", [](SgMaterial& self, const Vector3f c){ self.setDiffuseColor(c); })
        .def_property("emissiveColor",
                      &SgMaterial::emissiveColor,
                      [](SgMaterial& self, const Vector3f c){ self.setEmissiveColor(c); })
        .def("setEmissiveColor", [](SgMaterial& self, const Vector3f c){ self.setEmissiveColor(c); })
        .def_property("shininess", &SgMaterial::shininess, &SgMaterial::setShininess)
        .def("setShininess", &SgMaterial::setShininess)
        .def_property("specularColor",
                      &SgMaterial::specularColor,
                      [](SgMaterial& self, const Vector3f c){ self.setSpecularColor(c); })
        .def("setSpecularColor", [](SgMaterial& self, const Vector3f c){ self.setSpecularColor(c); })
        .def_property("transparency", &SgMaterial::transparency, &SgMaterial::setTransparency)
        .def("setTransparency", &SgMaterial::setTransparency)
        ;

    py::class_<SgMeshBase, SgMeshBasePtr, SgObject>(m, "SgMeshBase");

    py::class_<SgMesh, SgMeshPtr, SgMeshBase>(m, "SgMesh");

    py::class_<SgShape, SgShapePtr, SgNode>(m, "SgShape")
        .def(py::init<>())
        .def_property("mesh", (SgMesh* (SgShape::*)()) &SgShape::mesh, &SgShape::setMesh)
        .def("setMesh", &SgShape::setMesh)
        .def("getOrCreateMesh", &SgShape::getOrCreateMesh)
        .def_property("material", (SgMaterial* (SgShape::*)()) &SgShape::material, &SgShape::setMaterial)
        .def("setMaterial", &SgShape::setMaterial)
        .def("getOrCreateMaterial", &SgShape::getOrCreateMaterial)
        ;
}

}

        
    
    

        
        



