/*!
 * @author Shin'ichiro Nakaoka
*/

#include "PyBase.h"
#include "../ItemTreeView.h"
#include "../RootItem.h"
#include <cnoid/PySignal>

using namespace cnoid;
namespace py = boost::python;

// for MSVC++2015 Update3
CNOID_PYTHON_DEFINE_GET_POINTER(ItemTreeView)

namespace {

RootItemPtr ItemTreeView_rootItem(ItemTreeView& self) { return self.rootItem(); }

py::object ItemTreeView_selectedItems(ItemTreeView& self, py::object itemClass){
    return getPyNarrowedItemList(self.selectedItems(), itemClass);
}

py::object ItemTreeView_selectedItem(ItemTreeView& self, py::object itemClass){
    return getPyNarrowedFirstItem(self.selectedItems(), itemClass);
}

py::object ItemTreeView_selectedSubItems(ItemTreeView& self, ItemPtr topItem, py::object itemClass){
    return getPyNarrowedItemList(self.selectedSubItems<Item>(topItem), itemClass);
}

py::object ItemTreeView_selectedSubItem(ItemTreeView& self, ItemPtr topItem, py::object itemClass){
    return getPyNarrowedFirstItem(self.selectedSubItems<Item>(topItem), itemClass);
}

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(ItemTreeView_selectItem_overloads, selectItem, 1, 2)

ItemList<Item> ItemTreeView_checkedItems1(ItemTreeView& self) {
    return self.checkedItems<Item>();
}

ItemList<Item> ItemTreeView_checkedItems2(ItemTreeView& self, int id) {
    return self.checkedItems<Item>(id);
}

/*
py::object ItemTreeView_checkedItems3(ItemTreeView& self, py::object itemClass) {
    return getPyNarrowedItemList(self.checkedItems<Item>(), itemClass);
}

py::object ItemTreeView_checkedItems4(ItemTreeView& self, py::object itemClass, int id) {
    return getPyNarrowedItemList(self.checkedItems<Item>(id), itemClass);
}
*/


bool ItemTreeView_isItemChecked1(ItemTreeView& self, ItemPtr item){
    return self.isItemChecked(item);
}

bool ItemTreeView_isItemChecked2(ItemTreeView& self, ItemPtr item, int id){
    return self.isItemChecked(item, id);
}

bool ItemTreeView_checkItem1(ItemTreeView& self, ItemPtr item)
{
    return self.checkItem(item);
}

bool ItemTreeView_checkItem2(ItemTreeView& self, ItemPtr item, bool check)
{
    return self.checkItem(item, check);
}

bool ItemTreeView_checkItem3(ItemTreeView& self, ItemPtr item, bool check, int id)
{
    return self.checkItem(item, check, id);
}

SignalProxy<void(Item*, bool)> (ItemTreeView::*ItemTreeView_sigCheckToggled1)(int) = &ItemTreeView::sigCheckToggled;
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(ItemTreeView_sigCheckToggled1_overloads, sigCheckToggled, 0, 1)
SignalProxy<void(bool)> (ItemTreeView::*ItemTreeView_sigCheckToggled2)(Item*, int) = &ItemTreeView::sigCheckToggled;
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(ItemTreeView_sigCheckToggled2_overloads, sigCheckToggled, 1, 2)

}

namespace cnoid {

void exportPyItemTreeView()
{
    PySignal<void(const ItemList<>&)>("ItemListSignal");
    PySignal<void(Item* item, bool isChecked)>("ItemBoolSignal");
    
    py::class_<ItemTreeView, ItemTreeView*, py::bases<View>, boost::noncopyable>("ItemTreeView", py::no_init)
        .def("instance", &ItemTreeView::instance, py::return_value_policy<py::reference_existing_object>()).staticmethod("instance")
        .def("getInstance", &ItemTreeView::instance, py::return_value_policy<py::reference_existing_object>()).staticmethod("getInstance")
        .def("rootItem", ItemTreeView_rootItem)
        .def("getRootItem", ItemTreeView_rootItem)
        .def("selectedItems", &ItemTreeView::selectedItems<Item>)
        .def("getSelectedItems", &ItemTreeView::selectedItems<Item>)
        .def("selectedItems", ItemTreeView_selectedItems)
        .def("selectedItem", ItemTreeView_selectedItem)
        .def("selectedSubItems", ItemTreeView_selectedSubItems)
        .def("selectedSubItem", ItemTreeView_selectedSubItem)
        .def("isItemSelected", &ItemTreeView::isItemSelected)
        .def("selectItem", &ItemTreeView::selectItem, ItemTreeView_selectItem_overloads())
        .def("selectAllItems", &ItemTreeView::selectAllItems)
        .def("clearSelection", &ItemTreeView::clearSelection)
        .def("checkedItems", ItemTreeView_checkedItems1)
        .def("getCheckedItems", ItemTreeView_checkedItems1)
        .def("checkedItems", ItemTreeView_checkedItems2)
        .def("getCheckedItems", ItemTreeView_checkedItems2)
        //.def("checkedItems", ItemTreeView_checkedItems3)
        //.def("checkedItems", ItemTreeView_checkedItems4)
        .def("isItemChecked", ItemTreeView_isItemChecked1)
        .def("isItemChecked", ItemTreeView_isItemChecked2)
        .def("checkItem", ItemTreeView_checkItem1)
        .def("checkItem", ItemTreeView_checkItem2)
        .def("checkItem", ItemTreeView_checkItem3)
        .def("sigSelectionChanged", &ItemTreeView::sigSelectionChanged)
        .def("getSigSelectionChanged", &ItemTreeView::sigSelectionChanged)
        .def("sigSelectionOrTreeChanged", &ItemTreeView::sigSelectionOrTreeChanged)
        .def("getSigSelectionOrTreeChanged", &ItemTreeView::sigSelectionOrTreeChanged)
        .def("sigCheckToggled", ItemTreeView_sigCheckToggled1, ItemTreeView_sigCheckToggled1_overloads())
        .def("getSigCheckToggled", ItemTreeView_sigCheckToggled1, ItemTreeView_sigCheckToggled1_overloads())
        .def("sigCheckToggled", ItemTreeView_sigCheckToggled2, ItemTreeView_sigCheckToggled2_overloads())
        .def("getSigCheckToggled", ItemTreeView_sigCheckToggled2, ItemTreeView_sigCheckToggled2_overloads())
        .def("cutSelectedItems", &ItemTreeView::cutSelectedItems)
        ;
}

}
