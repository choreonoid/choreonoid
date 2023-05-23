#include "PyUtil.h"
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
        .def_property("specularExponent", &SgMaterial::specularExponent, &SgMaterial::specularExponent)
        .def("setSpecularExponent", &SgMaterial::setSpecularExponent)
        .def_property("specularColor",
                      &SgMaterial::specularColor,
                      [](SgMaterial& self, const Vector3f c){ self.setSpecularColor(c); })
        .def("setSpecularColor", [](SgMaterial& self, const Vector3f c){ self.setSpecularColor(c); })
        .def_property("transparency", &SgMaterial::transparency, &SgMaterial::setTransparency)
        .def("setTransparency", &SgMaterial::setTransparency)
        ;

    py::class_<SgMeshBase, SgMeshBasePtr, SgObject>(m, "SgMeshBase")
        .def("boundingBox", [](SgMeshBase &self) { BoundingBox ret = self.boundingBox(); return ret; })
        .def("updateBoundingBox", &SgMeshBase::updateBoundingBox)
        .def("hasVertices", &SgMeshBase::hasVertices)
        .def("vertices", [](SgMeshBase &self) {
             SgVertexArray *va_ = self.vertices();
             Eigen::Matrix<float, -1, -1, Eigen::RowMajor> rm(va_->size(), 3);
             int cntr = 0;
             for(auto it = va_->begin(); it != va_->end(); it++, cntr++) rm.row(cntr) = *it;
             return rm; })
        .def("hasNormals", &SgMeshBase::hasNormals)
        .def("normals", [](SgMeshBase &self) {
             SgNormalArray *nm_ = self.normals();
             Eigen::Matrix<float, -1, -1, Eigen::RowMajor> rm(nm_->size(), 3);
             int cntr = 0;
             for(auto it = nm_->begin(); it != nm_->end(); it++, cntr++) rm.row(cntr) = *it;
             return rm; })
        .def("hasColors", &SgMeshBase::hasColors)
        .def("colors", [](SgMeshBase &self) {
             SgColorArray *col_ = self.colors();
             Eigen::Matrix<float, -1, -1, Eigen::RowMajor> rm(col_->size(), 3);
             int cntr = 0;
             for(auto it = col_->begin(); it != col_->end(); it++, cntr++) rm.row(cntr) = *it;
             return rm; })
        .def("hasTexCoords", &SgMeshBase::hasTexCoords)
        .def("texCoords", [](SgMeshBase &self) {
             SgTexCoordArray *tc_ = self.texCoords();
             Eigen::Matrix<float, -1, -1, Eigen::RowMajor> rm(tc_->size(), 2);
             int cntr = 0;
             for(auto it = tc_->begin(); it != tc_->end(); it++, cntr++) rm.row(cntr) = *it;
             return rm; })
        .def("hasFaceVertexIndices", &SgMeshBase::hasFaceVertexIndices)
        .def("faceVertexIndices", (SgIndexArray&(SgMeshBase::*)())&SgMeshBase::faceVertexIndices)
        //.def("faceVertexIndices", [](SgMeshBase &self) { SgIndexArray idx_ = self.faceVertexIndices(); return idx_; })
        .def("hasNormalIndices", &SgMeshBase::hasNormalIndices)
        .def("normalIndices", (SgIndexArray&(SgMeshBase::*)())&SgMeshBase::normalIndices)
        //.def("normalIndices", [](SgMeshBase &self) { SgIndexArray idx_ = self.normalIndices(); return idx_; })
        .def("hasColorIndices", &SgMeshBase::hasColorIndices)
        .def("colorIndices", (SgIndexArray&(SgMeshBase::*)())&SgMeshBase::colorIndices)
        //.def("colorIndices", [](SgMeshBase &self) { SgIndexArray idx_ = self.colorIndices(); return idx_; })
        .def("hasTexCoordIndices", &SgMeshBase::hasTexCoordIndices)
        .def("texCoordIndices", (SgIndexArray&(SgMeshBase::*)())&SgMeshBase::texCoordIndices)
        //.def("texCoordIndices", [](SgMeshBase &self) { SgIndexArray idx_ = self.texCoordIndices(); return idx_; })
        .def("creaseAngle", &SgMeshBase::creaseAngle)
        .def("isSolid", &SgMeshBase::isSolid)
        ;

    py::class_<SgMesh, SgMeshPtr, SgMeshBase> sgMesh(m, "SgMesh");

    py::enum_<SgMesh::PrimitiveType>(sgMesh, "PrimitiveType")
        .value("MeshType", SgMesh::MeshType)
        .value("BoxType", SgMesh::BoxType)
        .value("SphereType", SgMesh::SphereType)
        .value("CylinderType", SgMesh::CylinderType)
        .value("ConeType", SgMesh::ConeType)
        .value("CapsuleType", SgMesh::CapsuleType)
        .export_values();

    py::enum_<SgMesh::ExtraDivisionMode>(sgMesh, "ExtraDivisionMode")
        .value("ExtraDivisionPreferred", SgMesh::ExtraDivisionPreferred)
        .value("ExtraDivisionX", SgMesh::ExtraDivisionX)
        .value("ExtraDivisionY", SgMesh::ExtraDivisionY)
        .value("ExtraDivisionZ", SgMesh::ExtraDivisionZ)
        .value("ExtraDivisionAll", SgMesh::ExtraDivisionAll)
        .export_values();

    sgMesh
        .def(py::init<>())
        .def("updateBoundingBox", &SgMesh::updateBoundingBox)
        .def("triangleVertices", (SgIndexArray&(SgMesh::*)())&SgMesh::triangleVertices)
        //.def("triangleVertices", [](SgMesh &self) { SgIndexArray idx_ = self.triangleVertices(); return idx_; })
        .def("hasTriangles", &SgMesh::hasTriangles)
        .def("numTriangles", &SgMesh::numTriangles)
        .def("triangle", [](SgMesh &self, int index) { Array3i ret = self.triangle(index); return ret; })
        .def("primitiveType", &SgMesh::primitiveType)
        //.def("primitive", &SgMesh::primitive)
        ;
    
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

        
    
    

        
        



