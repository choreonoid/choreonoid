/*!
  @author Shin'ichiro Nakaoka
*/

#include "../SceneDragger.h"
#include "../PositionDragger.h"
#include <cnoid/Py3Util>

namespace py=pybind11;
using namespace cnoid;

namespace cnoid {

void exportPySceneTypes(py::module m)
{
    py::object sceneDraggerClass =
        py::class_<SceneDragger, SceneDraggerPtr, SgPosTransform>(m, "SceneDragger")
        .def("isContainerMode", &SceneDragger::isContainerMode)
        .def("setContainerMode", &SceneDragger::setContainerMode)
        .def("isDragging", &SceneDragger::isDragging)
        .def("draggedPosition", &SceneDragger::draggedPosition)
        ;

    py::implicitly_convertible<SceneDraggerPtr, SgPosTransformPtr>();
    
    py::object positionDraggerClass =
        py::class_<PositionDragger, PositionDraggerPtr, SceneDragger>(m, "PositionDragger")
        .def("setDraggableAxes", &PositionDragger::setDraggableAxes)
        .def("draggableAxes", &PositionDragger::draggableAxes)
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
    
    py::implicitly_convertible<PositionDraggerPtr, SceneDraggerPtr>();
}

}
