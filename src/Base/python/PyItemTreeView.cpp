#include "PyItemList.h"
#include "PyQString.h"
#include "../ItemTreeView.h"
#include <cnoid/PyUtil>
#include <cnoid/PySignal>

using namespace cnoid;
namespace nb = nanobind;

namespace cnoid {

void exportPyItemTreeView(nb::module_ m)
{
    PySignal<void(const ItemList<>&)>(m, "ItemListSignal");
    PySignal<void(Item* item, bool isChecked)>(m, "ItemBoolSignal");

    nb::class_<ItemTreeView, View>(m, "ItemTreeView")
        .def_prop_ro_static("instance", [](nb::handle){ return ItemTreeView::instance(); }, nb::rv_policy::reference)
        .def("setExpanded", &ItemTreeView::setExpanded, nb::arg("item"), nb::arg("on") = true)
        ;
}

}
