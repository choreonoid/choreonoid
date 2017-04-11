/*!
 * @author Shin'ichiro Nakaoka
*/

#include "PyBase.h"
#include "../ItemTreeView.h"
#include "../RootItem.h"
#include <cnoid/Py3Signal>

namespace py = pybind11;
using namespace cnoid;

// for MSVC++2015 Update3
CNOID_PYTHON_DEFINE_GET_POINTER(ItemTreeView)

namespace cnoid {

void exportPyItemTreeView(py::module m)
{
    PySignal<void(const ItemList<>&)>(m, "ItemListSignal");
    PySignal<void(Item* item, bool isChecked)>(m, "ItemBoolSignal");
    
    py::class_<ItemTreeView, View>(m, "ItemTreeView")
        .def_static("instance", &ItemTreeView::instance, py::return_value_policy::reference)
        .def("rootItem", [](ItemTreeView& self) {
            return RootItemPtr(self.rootItem());
        })
        .def("showRoot", &ItemTreeView::showRoot)
        .def("selectedItems", &ItemTreeView::selectedItems<Item>)
        .def("selectedItems", [](ItemTreeView& self, py::object itemClass){
            return getPyNarrowedItemList(self.selectedItems(), itemClass);
        })
        .def("selectedItem", [](ItemTreeView& self, py::object itemClass){
            return getPyNarrowedFirstItem(self.selectedItems(), itemClass);
        })
        .def("selectedSubItems", [](ItemTreeView& self, ItemPtr topItem, py::object itemClass){
            return getPyNarrowedItemList(self.selectedSubItems<Item>(topItem), itemClass);
        })
        .def("selectedSubItem", [](ItemTreeView& self, ItemPtr topItem, py::object itemClass){
            return getPyNarrowedFirstItem(self.selectedSubItems<Item>(topItem), itemClass);
        })
        .def("isItemSelected", &ItemTreeView::isItemSelected)
        .def("selectItem", &ItemTreeView::selectItem, py::arg("item"), py::arg("select")=true)
        .def("selectAllItems", &ItemTreeView::selectAllItems)
        .def("clearSelection", &ItemTreeView::clearSelection)
        .def("checkedItems", [](ItemTreeView& self,int id = 0) {
            return self.checkedItems<Item>();
        })
        .def("isItemChecked", &ItemTreeView::isItemChecked, py::arg("item"), py::arg("id")=0)
        .def("checkItem", &ItemTreeView::checkItem, py::arg("item"), py::arg("check")=true, py::arg("id")=0)
        .def("sigSelectionChanged", &ItemTreeView::sigSelectionChanged)
        .def("sigSelectionOrTreeChanged", &ItemTreeView::sigSelectionOrTreeChanged)
        .def("sigCheckToggled", (SignalProxy<void(Item*, bool)> (ItemTreeView::*)(int))
                &ItemTreeView::sigCheckToggled, py::arg("id")=0 )
        .def("cutSelectedItems", &ItemTreeView::cutSelectedItems)
        ;
}

}
