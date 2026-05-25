#include "../PositionDragger.h"
#include <cnoid/PyUtil>

using namespace cnoid;
namespace nb = nanobind;

namespace cnoid {

void exportPySceneTypes(nb::module_ m)
{
    nb::class_<PositionDragger, SgPosTransform> positionDraggerClass(m, "PositionDragger");
    positionDraggerClass
        .def_prop_rw("draggableAxes", &PositionDragger::draggableAxes, &PositionDragger::setDraggableAxes)
        .def("setDraggableAxes", &PositionDragger::setDraggableAxes)
        .def("isContainerMode", &PositionDragger::isContainerMode)
        .def("setContainerMode", &PositionDragger::setContainerMode)
        .def("isDragging", &PositionDragger::isDragging)
        .def_prop_ro("draggingPosition", &PositionDragger::draggingPosition)
        .def_prop_ro("globalDraggingPosition", &PositionDragger::globalDraggingPosition)
        ;

    positionDraggerClass.attr("TX") = (int)PositionDragger::TX;
    positionDraggerClass.attr("TY") = (int)PositionDragger::TY;
    positionDraggerClass.attr("TZ") = (int)PositionDragger::TZ;
    positionDraggerClass.attr("TRANSLATION_AXES") = (int)PositionDragger::TRANSLATION_AXES;
    positionDraggerClass.attr("RX") = (int)PositionDragger::RX;
    positionDraggerClass.attr("RY") = (int)PositionDragger::RY;
    positionDraggerClass.attr("RZ") = (int)PositionDragger::RZ;
    positionDraggerClass.attr("ROTATION_AXES") = (int)PositionDragger::ROTATION_AXES;
    positionDraggerClass.attr("ALL_AXES") = (int)PositionDragger::ALL_AXES;
}

}
