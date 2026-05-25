#include "PyUtil.h"
#include "PyEigenTypes.h"
#include "../MeshGenerator.h"
#include "../SceneDrawables.h"
#include <nanobind/eigen/dense.h>
#include <nanobind/stl/vector.h>

namespace nb = nanobind;
using namespace cnoid;

namespace cnoid {

void exportPyMeshUtils(nb::module_& m)
{
    nb::class_<MeshGenerator> meshGenerator(m, "MeshGenerator");

    nb::enum_<MeshGenerator::MeshOption>(meshGenerator, "MeshOption", nb::is_arithmetic())
        .value("NoOption", MeshGenerator::NoOption)
        .value("TextureCoordinate", MeshGenerator::TextureCoordinate)
        .export_values();

    meshGenerator
        .def(nb::init<>())
        .def_prop_rw("divisionNumber", &MeshGenerator::divisionNumber, &MeshGenerator::setDivisionNumber)
        .def("setDivisionNumber", &MeshGenerator::setDivisionNumber)
        .def_prop_ro_static("defaultDivisionNumber", [](nb::handle){ return MeshGenerator::defaultDivisionNumber(); })
        .def("setExtraDivisionNumber", &MeshGenerator::setExtraDivisionNumber,
             nb::arg("n"), nb::arg("flags") = SgMesh::ExtraDivisionPreferred)
        .def_prop_ro("extraDivisionNumber", &MeshGenerator::extraDivisionNumber)
        .def_prop_ro("extraDivisionMode", &MeshGenerator::extraDivisionMode)
        .def("setNormalGenerationEnabled", &MeshGenerator::setNormalGenerationEnabled)
        .def("isNormalGenerationEnabled", &MeshGenerator::isNormalGenerationEnabled)
        .def("setBoundingBoxUpdateEnabled", &MeshGenerator::setBoundingBoxUpdateEnabled)
        .def("isBoundingBoxUpdateEnabled", &MeshGenerator::isBoundingBoxUpdateEnabled)
        .def("generateBox",
             [](MeshGenerator& self, const python::Vector3Arg& size, int options){
                 return self.generateBox(size.value, options); },
             nb::arg("size"), nb::arg("options") = MeshGenerator::NoOption)
        .def("generateSphere", (SgMesh*(MeshGenerator::*)(double, int)) &MeshGenerator::generateSphere,
             nb::arg("radius"), nb::arg("options") = MeshGenerator::NoOption)
        .def("generateCylinder", (SgMesh*(MeshGenerator::*)(double, double, int)) &MeshGenerator::generateCylinder,
             nb::arg("radius"), nb::arg("height"), nb::arg("options") = MeshGenerator::NoOption)
        .def("generateCone", (SgMesh*(MeshGenerator::*)(double, double, int)) &MeshGenerator::generateCone,
             nb::arg("radius"), nb::arg("height"), nb::arg("options") = MeshGenerator::NoOption)
        .def("generateCapsule", (SgMesh*(MeshGenerator::*)(double, double)) &MeshGenerator::generateCapsule)
        .def("generateDisc", &MeshGenerator::generateDisc)
        .def("generateArrow", &MeshGenerator::generateArrow)
        .def("generateTorus", (SgMesh* (MeshGenerator::*)(double, double)) &MeshGenerator::generateTorus)
        .def("generateTorus", (SgMesh* (MeshGenerator::*)(double, double, double, double)) &MeshGenerator::generateTorus)
        .def("generateExtrusion", &MeshGenerator::generateExtrusion,
             nb::arg("extrusion"), nb::arg("enableTextureCoordinate") = false)
        .def("generateElevationGrid", &MeshGenerator::generateElevationGrid,
             nb::arg("elevationGrid"), nb::arg("enableTextureCoordinate") = false)
        ;

    nb::class_<MeshGenerator::Extrusion>(meshGenerator, "Extrusion")
        .def(nb::init<>())
        .def_rw("crossSection", &MeshGenerator::Extrusion::crossSection)
        .def_rw("spine", &MeshGenerator::Extrusion::spine)
        .def_rw("orientation", &MeshGenerator::Extrusion::orientation)
        .def_rw("scale", &MeshGenerator::Extrusion::scale)
        .def_rw("creaseAngle", &MeshGenerator::Extrusion::creaseAngle)
        .def_rw("beginCap", &MeshGenerator::Extrusion::beginCap)
        .def_rw("endCap", &MeshGenerator::Extrusion::endCap)
        ;

    nb::class_<MeshGenerator::ElevationGrid>(meshGenerator, "ElevationGrid")
        .def(nb::init<>())
        .def_rw("xDimension", &MeshGenerator::ElevationGrid::xDimension)
        .def_rw("zDimension", &MeshGenerator::ElevationGrid::zDimension)
        .def_rw("xSpacing", &MeshGenerator::ElevationGrid::xSpacing)
        .def_rw("zSpacing", &MeshGenerator::ElevationGrid::zSpacing)
        .def_rw("height", &MeshGenerator::ElevationGrid::height)
        .def_rw("ccw", &MeshGenerator::ElevationGrid::ccw)
        .def_rw("creaseAngle", &MeshGenerator::ElevationGrid::creaseAngle)
        ;
}

}
