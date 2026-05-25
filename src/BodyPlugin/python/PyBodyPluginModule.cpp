#include <cnoid/PyUtil>

using namespace cnoid;
namespace nb = nanobind;

namespace cnoid {

void exportBodyItem(nb::module_ m);
void exportItems(nb::module_ m);
void exportSimulationClasses(nb::module_ m);

}

NB_MODULE(BodyPlugin, m)
{
    m.doc() = "Choreonoid BodyPlugin module";

    nb::module_::import_("cnoid.Base");
    nb::module_::import_("cnoid.Body");

    exportBodyItem(m);
    exportItems(m);
    exportSimulationClasses(m);
}
