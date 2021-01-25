/*!
  @author Shin'ichiro Nakaoka
*/

#include "../PositionDragger.h"
#include <cnoid/PyUtil>

using namespace cnoid;
namespace py = pybind11;

namespace cnoid {

void exportPySceneTypes(py::module m)
{
    py::object positionDraggerClass =
        py::class_<PositionDragger, PositionDraggerPtr, SgPosTransform>(
            m, "PositionDragger", py::multiple_inheritance())
        .def_property("draggableAxes", &PositionDragger::draggableAxes, &PositionDragger::setDraggableAxes)
        .def("setDraggableAxes", &PositionDragger::setDraggableAxes)
        .def("isContainerMode", &PositionDragger::isContainerMode)
        .def("setContainerMode", &PositionDragger::setContainerMode)
        .def("isDragging", &PositionDragger::isDragging)
        .def_property_readonly("draggingPosition", &PositionDragger::draggingPosition)
        .def_property_readonly("globalDraggingPosition", &PositionDragger::globalDraggingPosition)
        .def_property_readonly("draggedPosition", &PositionDragger::globalDraggingPosition) // deprecated

        // deprecated
        .def("getDraggableAxes", &PositionDragger::draggableAxes)
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
