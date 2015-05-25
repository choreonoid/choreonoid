/*!
  @author Shin'ichiro Nakaoka
*/

#include "../SceneDragger.h"
#include <cnoid/PyUtil>

using namespace boost;
using namespace boost::python;
using namespace cnoid;

namespace {

int PositionDragger_TX() { return PositionDragger::TX; }

}

namespace cnoid {

void exportPySceneTypes()
{
    object positionDraggerClass =
        class_<PositionDragger, PositionDraggerPtr, bases<SgPosTransform>, boost::noncopyable>("PositionDragger")
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
    
    implicitly_convertible<PositionDraggerPtr, SgPosTransformPtr>();
}

}
