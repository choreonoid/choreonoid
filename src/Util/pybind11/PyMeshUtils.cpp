#include "PyUtil.h"
#include "../MeshGenerator.h"

using namespace cnoid;
namespace py = pybind11;

namespace cnoid {

void exportPyMeshUtils(py::module& m)
{
    py::class_<MeshGenerator> meshGenerator(m, "MeshGenerator");

    py::enum_<MeshGenerator::ExtraDivisionNumberFlag>(meshGenerator, "ExtraDivisionNumberFlag", py::arithmetic())
        .value("DivisionMax", MeshGenerator::DivisionMax)
        .value("DivisionX", MeshGenerator::DivisionX)
        .value("DivisionY", MeshGenerator::DivisionY)
        .value("DivisionZ", MeshGenerator::DivisionZ)
        .value("DivisionAll", MeshGenerator::DivisionAll)
        .export_values();

    meshGenerator
        .def(py::init<>())
        .def_property("divisionNumber", &MeshGenerator::divisionNumber, &MeshGenerator::setDivisionNumber)
        .def("setDivisionNumber", &MeshGenerator::setDivisionNumber)
        .def_property_readonly_static("defaultDivisionNumber", &MeshGenerator::defaultDivisionNumber)
        .def("setExtraDivisionNumber", &MeshGenerator::setExtraDivisionNumber,
             py::arg("n"), py::arg("flags") = MeshGenerator::DivisionMax)
        .def_property_readonly("extraDivisionNumber", &MeshGenerator::extraDivisionNumber)
        .def_property_readonly("extraDivisionNumberFlags", &MeshGenerator::extraDivisionNumberFlags)
        .def("setNormalGenerationEnabled", &MeshGenerator::setNormalGenerationEnabled)
        .def("isNormalGenerationEnabled", &MeshGenerator::isNormalGenerationEnabled)
        .def("setBoundingBoxUpdateEnabled", &MeshGenerator::setBoundingBoxUpdateEnabled)
        .def("isBoundingBoxUpdateEnabled", &MeshGenerator::isBoundingBoxUpdateEnabled)
        .def("generateBox", &MeshGenerator::generateBox,
             py::arg("size"), py::arg("enableTextureCoordinate") = false)
        .def("generateSphere", &MeshGenerator::generateSphere,
             py::arg("radius"), py::arg("enableTextureCoordinate") = false)
        .def("generateCylinder", &MeshGenerator::generateCylinder,
             py::arg("radius"), py::arg("height"), py::arg("bottom") = true, py::arg("top"),
             py::arg("side") = true, py::arg("enableTextureCoordinate") = false)
        .def("generateCone", &MeshGenerator::generateCone,
             py::arg("radius"), py::arg("height"),
             py::arg("bottom") = true, py::arg("side") = true, py::arg("enableTextureCoordinate") = false)
        .def("generateCapsule", &MeshGenerator::generateCapsule)
        .def("generateDisc", &MeshGenerator::generateDisc)
        .def("generateArrow", &MeshGenerator::generateArrow)
        .def("generateTorus", (SgMesh* (MeshGenerator::*)(double, double)) &MeshGenerator::generateTorus)
        .def("generateTorus", (SgMesh* (MeshGenerator::*)(double, double, double, double)) &MeshGenerator::generateTorus)
        .def("generateExtrusion", &MeshGenerator::generateExtrusion,
             py::arg("extrusion"), py::arg("enableTextureCoordinate") = false)
        .def("generateElevationGrid", &MeshGenerator::generateElevationGrid,
             py::arg("elevationGrid"), py::arg("enableTextureCoordinate") = false)
        ;

    py::class_<MeshGenerator::Extrusion>(meshGenerator, "Extrusion")
        .def(py::init<>())
        .def_readwrite("crossSection", &MeshGenerator::Extrusion::crossSection)
        .def_readwrite("spine", &MeshGenerator::Extrusion::spine)
        .def_readwrite("orientation", &MeshGenerator::Extrusion::orientation)
        .def_readwrite("scale", &MeshGenerator::Extrusion::scale)
        .def_readwrite("creaseAngle", &MeshGenerator::Extrusion::creaseAngle)
        .def_readwrite("beginCap", &MeshGenerator::Extrusion::beginCap)
        .def_readwrite("endCap", &MeshGenerator::Extrusion::endCap)
        ;

    py::class_<MeshGenerator::ElevationGrid>(meshGenerator, "ElevationGrid")
        .def(py::init<>())
        .def_readwrite("xDimension", &MeshGenerator::ElevationGrid::xDimension)
        .def_readwrite("zDimension", &MeshGenerator::ElevationGrid::zDimension)
        .def_readwrite("xSpacing", &MeshGenerator::ElevationGrid::xSpacing)
        .def_readwrite("zSpacing", &MeshGenerator::ElevationGrid::zSpacing)
        .def_readwrite("height", &MeshGenerator::ElevationGrid::height)
        .def_readwrite("ccw", &MeshGenerator::ElevationGrid::ccw)
        .def_readwrite("creaseAngle", &MeshGenerator::ElevationGrid::creaseAngle)
        ;
}

}
