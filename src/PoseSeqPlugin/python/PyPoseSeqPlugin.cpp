#include "../PoseSeqItem.h"
#include <cnoid/PyBase>

using namespace cnoid;
namespace nb = nanobind;

NB_MODULE(PoseSeqPlugin, m)
{
    m.doc() = "Choreonoid PoseSeqPlugin module";

    nb::module_::import_("cnoid.BodyPlugin");

    nb::class_<PoseSeqItem, Item>(m, "PoseSeqItem");
}
