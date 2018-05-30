/*!
  @author Shin'ichiro Nakaoka
*/

#include "../SceneDragger.h"
#include "../PositionDragger.h"
#include <cnoid/PyUtil>

using namespace boost;
using namespace boost::python;
using namespace cnoid;

namespace cnoid {

void exportPySceneTypes()
{
    object sceneDraggerClass =
        class_<SceneDragger, SceneDraggerPtr, bases<SgPosTransform>, boost::noncopyable>("SceneDragger", no_init)
        .def("isContainerMode", &SceneDragger::isContainerMode)
        .def("setContainerMode", &SceneDragger::setContainerMode)
        .def("isDragging", &SceneDragger::isDragging)
        .def("draggedPosition", &SceneDragger::draggedPosition)
        .def("getDraggedPosition", &SceneDragger::draggedPosition)
        ;

    implicitly_convertible<SceneDraggerPtr, SgPosTransformPtr>();
    
    object positionDraggerClass =
        class_<PositionDragger, PositionDraggerPtr, bases<SceneDragger>, boost::noncopyable>("PositionDragger")
        .def("setDraggableAxes", &PositionDragger::setDraggableAxes)
        .def("draggableAxes", &PositionDragger::draggableAxes)
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
    
    implicitly_convertible<PositionDraggerPtr, SceneDraggerPtr>();
}

}
