#include "PyUtil.h"
#include "PyEigenTypes.h"
#include "../PolyhedralRegion.h"
#include "../BoundingBox.h"
#include <nanobind/eigen/dense.h>

namespace nb = nanobind;
using namespace cnoid;

namespace cnoid {

void exportPyGeometryTypes(nb::module_& m)
{
    nb::class_<PolyhedralRegion>(m, "PolyhedralRegion")
        .def(nb::init<>())
        .def_prop_ro("numBoundingPlanes", &PolyhedralRegion::numBoundingPlanes)
        .def("clear", &PolyhedralRegion::clear)
        .def("addBoundingPlane",
             [](PolyhedralRegion& self, const python::Vector3Arg& normal, const python::Vector3Arg& point){
                 self.addBoundingPlane(normal.value, point.value); })
        .def("checkInside",
             [](PolyhedralRegion& self, const python::Vector3Arg& point){ return self.checkInside(point.value); })
        ;

    nb::class_<BoundingBox>(m, "BoundingBox")
        .def(nb::init<>())
        .def(nb::init<const BoundingBox&>())
        .def("clear", &BoundingBox::clear)
        .def("empty", &BoundingBox::empty)
        .def("boundingSphereRadius", &BoundingBox::boundingSphereRadius)
        .def("set",
             [](BoundingBox& self, const python::Vector3Arg& min, const python::Vector3Arg& max){
                 self.set(min.value, max.value); })
        .def("scale", &BoundingBox::scale)
        .def("transform", &BoundingBox::transform)
        .def("center", &BoundingBox::center)
        .def("size", &BoundingBox::size)
        .def("min", [](BoundingBox &self) { Vector3 ret = self.min(); return ret; })
        .def("max", [](BoundingBox &self) { Vector3 ret = self.max(); return ret; })
        ;
}

}
