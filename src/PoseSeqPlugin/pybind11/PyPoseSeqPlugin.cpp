/*!
  @author Shin'ichiro Nakaoka
*/

#include "../PoseSeqItem.h"
#include <cnoid/PyUtil>

namespace py = pybind11;
using namespace cnoid;

PYBIND11_PLUGIN(PoseSeqPlugin)
{
    py::module m("PoseSeqPlugin", "PoseSeqPlugin Python Module");

    py::class_< PoseSeqItem, PoseSeqItemPtr, Item >(m, "PoseSeqItem");

    return m.ptr();
}
