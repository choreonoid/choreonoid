/*!
  @author Shin'ichiro Nakaoka
*/

#include "PyUtil.h"
#include "../PolyhedralRegion.h"

using namespace boost::python;
using namespace cnoid;

namespace cnoid {

void exportPyGeometryTypes()
{
    class_<PolyhedralRegion>("PolyhedralRegion")
        .def("numBoundingPlanes", &PolyhedralRegion::numBoundingPlanes)
        .def("getNumBoundingPlanes", &PolyhedralRegion::numBoundingPlanes)
        .def("clear", &PolyhedralRegion::clear)
        .def("addBoundingPlane", &PolyhedralRegion::addBoundingPlane)
        .def("checkInside", &PolyhedralRegion::checkInside)
        ;
}

}
