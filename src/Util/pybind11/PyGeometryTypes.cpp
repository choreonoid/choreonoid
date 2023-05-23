/*!
  @author Shin'ichiro Nakaoka
*/

#include "PyUtil.h"
#include "../PolyhedralRegion.h"
#include "../BoundingBox.h"

namespace py = pybind11;
using namespace cnoid;

namespace cnoid {

void exportPyGeometryTypes(py::module& m)
{
    py::class_<PolyhedralRegion>(m, "PolyhedralRegion")
        .def(py::init())
        .def_property_readonly("numBoundingPlanes", &PolyhedralRegion::numBoundingPlanes)
        .def("clear", &PolyhedralRegion::clear)
        .def("addBoundingPlane", &PolyhedralRegion::addBoundingPlane)
        .def("checkInside", &PolyhedralRegion::checkInside)

        // deprecated
        .def("getNumBoundingPlanes", &PolyhedralRegion::numBoundingPlanes)
        ;

    py::class_<BoundingBox>(m, "BoundingBox")
        .def(py::init())
        .def(py::init<const BoundingBox&>())
        .def("clear", &BoundingBox::clear)
        .def("empty", &BoundingBox::empty)
        .def("boundingSphereRadius", &BoundingBox::boundingSphereRadius)
        .def("set", &BoundingBox::set)
        .def("scale", &BoundingBox::scale)
        .def("transform", &BoundingBox::transform)
        .def("center", &BoundingBox::center)
        .def("size", &BoundingBox::size)
        .def("min", [](BoundingBox &self) { Vector3 ret = self.min(); return ret; })
        .def("max", [](BoundingBox &self) { Vector3 ret = self.max(); return ret; })
        ;
}

}
