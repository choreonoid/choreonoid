/*!
  @author Shin'ichiro Nakaoka
*/

#include "../SceneDragger.h"
#include <cnoid/PyUtil>

using namespace boost;
using namespace boost::python;
using namespace cnoid;

namespace {

}

namespace cnoid {

void exportPySceneTypes()
{
    class_<PositionDragger, PositionDraggerPtr, bases<SgPosTransform>, boost::noncopyable>("PositionDragger")
        ;
    implicitly_convertible<PositionDraggerPtr, SgPosTransformPtr>();
}

}
