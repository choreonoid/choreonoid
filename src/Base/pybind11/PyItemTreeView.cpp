/*!
 * @author Shin'ichiro Nakaoka
*/

#include "PyQObjectHolder.h"
#include "PyItemList.h"
#include "PyQString.h"
#include "../ItemTreeView.h"
#include "../RootItem.h"
#include <cnoid/PyUtil>

using namespace cnoid;
namespace py = pybind11;

namespace cnoid {

void exportPyItemTreeView(py::module m)
{
    PySignal<void(const ItemList<>&)>(m, "ItemListSignal");
    PySignal<void(Item* item, bool isChecked)>(m, "ItemBoolSignal");
    
    py::class_<ItemTreeView, PyQObjectHolder<ItemTreeView>, View>(m, "ItemTreeView")
        .def_property_readonly_static("instance", [](py::object){ return ItemTreeView::instance(); })
        .def("setExpanded", &ItemTreeView::setExpanded, py::arg("item"), py::arg("on") = true)

        // deprecated
        .def("expandItem",
             [](ItemTreeView& self, Item* item, bool on){ self.setExpanded(item, on); },
             py::arg("item"), py::arg("on") = true)
        
        .def_property_readonly(
            "selectedItems", [](ItemTreeView&){ return RootItem::instance()->selectedItems();})
        .def("getSelectedItems", [](ItemTreeView&, py::object itemClass){
                return getPyNarrowedItemList(RootItem::instance()->selectedItems(), itemClass); })
        .def("getSelectedItem", [](ItemTreeView&, py::object itemClass){
                return getPyNarrowedFirstItem(RootItem::instance()->selectedItems(), itemClass); })
        .def("getSelectedSubItems", [](ItemTreeView&, ItemPtr topItem, py::object itemClass){
                return getPyNarrowedItemList(topItem->selectedDescendantItems<Item>(), itemClass); })
        .def("getSelectedSubItem", [](ItemTreeView&, ItemPtr topItem, py::object itemClass){
                return getPyNarrowedFirstItem(topItem->selectedDescendantItems<Item>(), itemClass); })
        .def("isItemSelected", [](ItemTreeView&, ItemPtr item){ return item->isSelected(); })
        .def("selectItem", [](ItemTreeView&, ItemPtr item, bool on){ item->setSelected(on); return true; },
             py::arg("item"), py::arg("on") = true)
        .def("selectAllItems", [](ItemTreeView&){ RootItem::instance()->setSubTreeItemsSelected(true); })
        .def("clearSelection", [](ItemTreeView&){ RootItem::instance()->setSubTreeItemsSelected(false); })
        .def("getCheckedItems",
             [](ItemTreeView&, int checkId){ return RootItem::instance()->checkedItems<Item>(checkId); },
             py::arg("checkId") = Item::PrimaryCheck)
        .def("isItemChecked",
             [](ItemTreeView&, ItemPtr item, int checkId){ return item->isChecked(checkId); },
             py::arg("item"), py::arg("checkId") = Item::PrimaryCheck)
        .def("checkItem",
             [](ItemTreeView&, ItemPtr item, bool on, int checkId){
                 item->setChecked(checkId, on); return true; },
             py::arg("item"), py::arg("on") = true, py::arg("checkId") = Item::PrimaryCheck)
        .def_property_readonly(
            "sigSelectionChanged", [](ItemTreeView&){ return RootItem::instance()->sigSelectedItemsChanged(); })
        .def_property_readonly(
            "sigCheckToggled", [](ItemTreeView&){ return RootItem::instance()->sigCheckToggled(); })
        .def("getSigCheckToggled",
             [](ItemTreeView&, int checkId){ return RootItem::instance()->sigCheckToggled(checkId); },
             py::arg("checkId") = Item::PrimaryCheck)
        .def(
            "getSigCheckToggled",
            [](ItemTreeView&, ItemPtr item, int checkId){ return item->sigCheckToggled(checkId); },
            py::arg("item"), py::arg("checkId") = Item::PrimaryCheck)
        ;
}

}
