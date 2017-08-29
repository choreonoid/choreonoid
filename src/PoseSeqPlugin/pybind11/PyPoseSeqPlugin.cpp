/*!
  @author Shin'ichiro Nakaoka
*/

#include "../PoseSeqItem.h"
#include <cnoid/PyBase>

using namespace cnoid;
namespace py = pybind11;

PYBIND11_MODULE(PoseSeqPlugin, m)
{
    m.doc() = "Choreonoid PoseSeqPlugin module";

    py::module::import("cnoid.BodyPlugin");

    py::class_<PoseSeqItem, PoseSeqItemPtr, Item>(m, "PoseSeqItem");
}
