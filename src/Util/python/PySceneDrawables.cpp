#include "PyUtil.h"
#include "PyNumpyHelper.h"
#include "PyEigenTypes.h"
#include "../SceneDrawables.h"
#include "../CloneMap.h"
#include <nanobind/eigen/dense.h>

using namespace cnoid;
using namespace cnoid::python;
namespace nb = nanobind;

namespace {

// Expose an SgVectorArray<Vector3f/2f> as a writable (N, ncols) NumPy view that
// shares memory with the array. 'owner' (the mesh that holds the array) keeps
// the backing memory alive while the view exists. Returns an empty (0, ncols)
// array when the array is null/empty (the field is not allocated yet).
template<class ArrayType>
nb::object meshVectorArrayView(ArrayType* a, int ncols, nb::handle owner)
{
    if(!a || a->empty()){
        float dummy = 0.0f;
        return bufferView2d<float>(&dummy, 0, ncols, nb::handle());
    }
    return bufferView2d<float>(a->data(), a->size(), ncols, owner);
}

// Expose an SgIndexArray (std::vector<int>) as a writable 1D NumPy view sharing
// memory. 'owner' keeps the object that holds the vector alive.
nb::object meshIndexArrayView(SgIndexArray& v, nb::handle owner)
{
    if(v.empty()){
        int dummy = 0;
        return bufferView1d<int>(&dummy, 0, nb::handle());
    }
    return bufferView1d<int>(v.data(), v.size(), owner);
}

}

namespace cnoid {

void exportPySceneDrawables(nb::module_& m)
{
    nb::class_<SgMaterial, SgObject>(m, "SgMaterial")
        .def(nb::init<>())
        .def_prop_rw("ambientIntensity", &SgMaterial::ambientIntensity, &SgMaterial::setAmbientIntensity)
        .def("setAmbientIntensity", &SgMaterial::setAmbientIntensity)
        .def_prop_rw("diffuseColor",
                      &SgMaterial::diffuseColor,
                      [](SgMaterial& self, const python::Vector3fArg& c){ self.setDiffuseColor(c.value); })
        .def("setDiffuseColor", [](SgMaterial& self, const python::Vector3fArg& c){ self.setDiffuseColor(c.value); })
        .def_prop_rw("emissiveColor",
                      &SgMaterial::emissiveColor,
                      [](SgMaterial& self, const python::Vector3fArg& c){ self.setEmissiveColor(c.value); })
        .def("setEmissiveColor", [](SgMaterial& self, const python::Vector3fArg& c){ self.setEmissiveColor(c.value); })
        .def_prop_rw("specularExponent", &SgMaterial::specularExponent, &SgMaterial::setSpecularExponent)
        .def("setSpecularExponent", &SgMaterial::setSpecularExponent)
        .def_prop_rw("specularColor",
                      &SgMaterial::specularColor,
                      [](SgMaterial& self, const python::Vector3fArg& c){ self.setSpecularColor(c.value); })
        .def("setSpecularColor", [](SgMaterial& self, const python::Vector3fArg& c){ self.setSpecularColor(c.value); })
        .def_prop_rw("transparency", &SgMaterial::transparency, &SgMaterial::setTransparency)
        .def("setTransparency", &SgMaterial::setTransparency)
        ;

    nb::class_<SgMeshBase, SgObject>(m, "SgMeshBase")
        .def("boundingBox", [](SgMeshBase &self) { BoundingBox ret = self.boundingBox(); return ret; })
        .def("updateBoundingBox", &SgMeshBase::updateBoundingBox)
        .def("hasVertices", &SgMeshBase::hasVertices)
        .def("vertices", [](SgMeshBase& self){ return meshVectorArrayView(self.vertices(), 3, nb::find(&self)); })
        .def("hasNormals", &SgMeshBase::hasNormals)
        .def("normals", [](SgMeshBase& self){ return meshVectorArrayView(self.normals(), 3, nb::find(&self)); })
        .def("hasColors", &SgMeshBase::hasColors)
        .def("colors", [](SgMeshBase& self){ return meshVectorArrayView(self.colors(), 3, nb::find(&self)); })
        .def("hasTexCoords", &SgMeshBase::hasTexCoords)
        .def("texCoords", [](SgMeshBase& self){ return meshVectorArrayView(self.texCoords(), 2, nb::find(&self)); })
        .def("hasFaceVertexIndices", &SgMeshBase::hasFaceVertexIndices)
        .def("faceVertexIndices", [](SgMeshBase& self){ return meshIndexArrayView(self.faceVertexIndices(), nb::find(&self)); })
        .def("hasNormalIndices", &SgMeshBase::hasNormalIndices)
        .def("normalIndices", [](SgMeshBase& self){ return meshIndexArrayView(self.normalIndices(), nb::find(&self)); })
        .def("hasColorIndices", &SgMeshBase::hasColorIndices)
        .def("colorIndices", [](SgMeshBase& self){ return meshIndexArrayView(self.colorIndices(), nb::find(&self)); })
        .def("hasTexCoordIndices", &SgMeshBase::hasTexCoordIndices)
        .def("texCoordIndices", [](SgMeshBase& self){ return meshIndexArrayView(self.texCoordIndices(), nb::find(&self)); })
        .def("creaseAngle", &SgMeshBase::creaseAngle)
        .def("isSolid", &SgMeshBase::isSolid)
        ;

    nb::class_<SgMesh, SgMeshBase> sgMesh(m, "SgMesh");

    nb::enum_<SgMesh::PrimitiveType>(sgMesh, "PrimitiveType", nb::is_arithmetic())
        .value("MeshType", SgMesh::MeshType)
        .value("BoxType", SgMesh::BoxType)
        .value("SphereType", SgMesh::SphereType)
        .value("CylinderType", SgMesh::CylinderType)
        .value("ConeType", SgMesh::ConeType)
        .value("CapsuleType", SgMesh::CapsuleType)
        .export_values();

    nb::enum_<SgMesh::ExtraDivisionMode>(sgMesh, "ExtraDivisionMode", nb::is_arithmetic())
        .value("ExtraDivisionPreferred", SgMesh::ExtraDivisionPreferred)
        .value("ExtraDivisionX", SgMesh::ExtraDivisionX)
        .value("ExtraDivisionY", SgMesh::ExtraDivisionY)
        .value("ExtraDivisionZ", SgMesh::ExtraDivisionZ)
        .value("ExtraDivisionAll", SgMesh::ExtraDivisionAll)
        .export_values();

    sgMesh
        .def(nb::init<>())
        .def("updateBoundingBox", &SgMesh::updateBoundingBox)
        .def("triangleVertices", [](SgMesh& self){ return meshIndexArrayView(self.triangleVertices(), nb::find(&self)); })
        .def("hasTriangles", &SgMesh::hasTriangles)
        .def("numTriangles", &SgMesh::numTriangles)
        .def("triangle", [](SgMesh &self, int index) { Array3i ret = self.triangle(index); return ret; })
        .def("primitiveType", &SgMesh::primitiveType)
        ;

    nb::class_<SgShape, SgNode>(m, "SgShape")
        .def(nb::init<>())
        .def_prop_rw("mesh", (SgMesh* (SgShape::*)()) &SgShape::mesh, &SgShape::setMesh)
        .def("setMesh", &SgShape::setMesh)
        .def("getOrCreateMesh", &SgShape::getOrCreateMesh)
        .def_prop_rw("material", (SgMaterial* (SgShape::*)()) &SgShape::material, &SgShape::setMaterial)
        .def("setMaterial", &SgShape::setMaterial)
        .def("getOrCreateMaterial", &SgShape::getOrCreateMaterial)
        ;
}

}
