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
        .def("selectItem", [](ItemTreeView& self, Item* item){ return self.selectItem(item); })
        .def("selectItem", [](ItemTreeView& self, Item* item, bool select){ return self.selectItem(item, select); })
        .def("selectAllItemss", &ItemTreeView::selectAllItems)
        .def("clearSelection", &ItemTreeView::clearSelection)
        .def("checkedItems", [](ItemTreeView& self){ return self.checkedItems<Item>(); })
        .def("checkedItems", [](ItemTreeView& self, int id){ return self.checkedItems<Item>(id); })
        .def("isItemChecked",[](ItemTreeView& self, Item* item){ return self.isItemChecked(item); })
        .def("isItemChecked",[](ItemTreeView& self, Item* item, int id){ return self.isItemChecked(item, id); })
        .def("checkItem",[](ItemTreeView& self, Item* item){ return self.checkItem(item); })
        .def("checkItem",[](ItemTreeView& self, Item* item, bool check){ return self.checkItem(item, check); })
        .def("checkItem",[](ItemTreeView& self, Item* item, bool check, int id){ return self.checkItem(item, check, id); })
        .def_property_readonly("sigSelectionChanged", &ItemTreeView::sigSelectionChanged)
        .def_property_readonly("sigCheckToggled", [](ItemTreeView& self){ return self.sigCheckToggled(); })
        .def("getSigCheckToggled", [](ItemTreeView& self){ return self.sigCheckToggled(); })
        .def("getSigCheckToggled", [](ItemTreeView& self, int id){ return self.sigCheckToggled(id); })
        .def("getSigCheckToggled", [](ItemTreeView& self, Item* item){ return self.sigCheckToggled(item); })
        .def("getSigCheckToggled", [](ItemTreeView& self, Item* item, int id){ return self.sigCheckToggled(item, id); })

        // deprecated
        .def_static("getInstance", &ItemTreeView::instance, py::return_value_policy::reference)
        .def("getSelectedItems", &ItemTreeView::selectedItems<Item>)
        .def("getSigSelectionChanged", &ItemTreeView::sigSelectionChanged)
        .def_property_readonly("sigCheckToggled", [](ItemTreeView& self, int id){ return self.sigCheckToggled(id); })
        .def_property_readonly("sigCheckToggled", [](ItemTreeView& self, Item* item){ return self.sigCheckToggled(item); })
        .def_property_readonly("sigCheckToggled", [](ItemTreeView& self, Item* item, int id){ return self.sigCheckToggled(item, id); })

        //.def_property_readonly("rootItem", &ItemTreeView::rootItem)
        //.def("getRootItem", &ItemTreeView::rootItem)

        ;
}

}
