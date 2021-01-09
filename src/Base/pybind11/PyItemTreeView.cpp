/*!
 * @author Shin'ichiro Nakaoka
*/

#include "PyItemList.h"
#include "PyQString.h"
#include "../ItemTreeView.h"
#include "../RootItem.h"
#include <cnoid/PySignal>
#include <cnoid/PyReferenced>

using namespace cnoid;
namespace py = pybind11;

namespace cnoid {

void exportPyItemTreeView(py::module m)
{
    PySignal<void(const ItemList<>&)>(m, "ItemListSignal");
    PySignal<void(Item* item, bool isChecked)>(m, "ItemBoolSignal");
    
    py::class_<ItemTreeView, View>(m, "ItemTreeView")
        .def_property_readonly_static(
            "instance", [](py::object){ return ItemTreeView::instance(); }, py::return_value_policy::reference)
        .def("setExpanded", &ItemTreeView::setExpanded, py::arg("item"), py::arg("on") = true)

        // deprecated
        .def("selectedItems", &ItemTreeView::selectedItems<Item>)
        .def("selectedItems", [](ItemTreeView& self, py::object itemClass){
                return getPyNarrowedItemList(self.selectedItems(), itemClass); })
        .def("selectedItem", [](ItemTreeView& self, py::object itemClass){
                return getPyNarrowedFirstItem(self.selectedItems(), itemClass); })
        .def("selectedSubItems", [](ItemTreeView& self, ItemPtr topItem, py::object itemClass){
                return getPyNarrowedItemList(self.selectedSubItems<Item>(topItem), itemClass); })
        .def("selectedSubItem", [](ItemTreeView& self, ItemPtr topItem, py::object itemClass){
                return getPyNarrowedFirstItem(self.selectedSubItems<Item>(topItem), itemClass); })
        .def("isItemSelected", &ItemTreeView::isItemSelected)
        .def("selectItem", &ItemTreeView::selectItem, py::arg("item"), py::arg("on") = true)
        .def("selectAllItemss", &ItemTreeView::selectAllItems)
        .def("clearSelection", &ItemTreeView::clearSelection)
        .def("checkedItems", &ItemTreeView::checkedItems<Item>, py::arg("checkId") = Item::PrimaryCheck)
        .def("isItemChecked", &ItemTreeView::isItemChecked, py::arg("item"), py::arg("checkId") = Item::PrimaryCheck)
        .def("checkItem",
             &ItemTreeView::checkItem, py::arg("item"), py::arg("on") = true, py::arg("checkId") = Item::PrimaryCheck)
        .def_property_readonly("sigSelectionChanged", &ItemTreeView::sigSelectionChanged)
        .def_property_readonly(
            "sigCheckToggled", [](ItemTreeView& self){ return self.sigCheckToggled(); })
        .def(
            "getSigCheckToggled",
            [](ItemTreeView& self, int checkId){ return self.sigCheckToggled(checkId); },
            py::arg("checkId") = Item::PrimaryCheck)
        .def(
            "getSigCheckToggled",
            [](ItemTreeView& self, Item* item, int checkId){ return self.sigCheckToggled(item, checkId); },
            py::arg("item"), py::arg("checkId") = Item::PrimaryCheck)
        .def_static("getInstance", &ItemTreeView::instance, py::return_value_policy::reference)
        .def("getSelectedItems", &ItemTreeView::selectedItems<Item>)
        .def("getSigSelectionChanged", &ItemTreeView::sigSelectionChanged)
        .def("expandItem",
             [](ItemTreeView& self, Item* item, bool on){ self.setExpanded(item, on); },
             py::arg("item"), py::arg("on") = true)
        ;
}

}
