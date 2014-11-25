/*!
 * @author Shin'ichiro Nakaoka
*/

#include "PyBase.h"
#include "../ItemTreeView.h"
#include "../RootItem.h"
#include <cnoid/PySignal>

using namespace boost;
using namespace boost::python;
using namespace cnoid;

namespace {

RootItemPtr ItemTreeView_rootItem(ItemTreeView& self) { return self.rootItem(); }

python::object ItemTreeView_selectedItems(ItemTreeView& self, python::object itemClass){
    return getPyNarrowedItemList(self.selectedItems(), itemClass);
}

python::object ItemTreeView_selectedItem(ItemTreeView& self, python::object itemClass){
    return getPyNarrowedFirstItem(self.selectedItems(), itemClass);
}

python::object ItemTreeView_selectedSubItems(ItemTreeView& self, ItemPtr topItem, python::object itemClass){
    return getPyNarrowedItemList(self.selectedSubItems<Item>(topItem), itemClass);
}

python::object ItemTreeView_selectedSubItem(ItemTreeView& self, ItemPtr topItem, python::object itemClass){
    return getPyNarrowedFirstItem(self.selectedSubItems<Item>(topItem), itemClass);
}

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(ItemTreeView_selectItem_overloads, selectItem, 1, 2)

ItemList<Item> ItemTreeView_checkedItems1(ItemTreeView& self, int id = 0) {
    return self.checkedItems<Item>(id);
}
BOOST_PYTHON_FUNCTION_OVERLOADS(ItemTreeView_checkedItems1_overloads, ItemTreeView_checkedItems1, 1, 2)

python::object ItemTreeView_checkedItems2(ItemTreeView& self, python::object itemClass, int id = 0) {
    return getPyNarrowedItemList(self.checkedItems<Item>(id), itemClass);
}
BOOST_PYTHON_FUNCTION_OVERLOADS(ItemTreeView_checkedItems2_overloads, ItemTreeView_checkedItems2, 2, 3)

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(ItemTreeView_isItemChecked_overloads, isItemChecked, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(ItemTreeView_checkItem_overloads, checkItem, 1, 3)
SignalProxy<void(Item*, bool)> (ItemTreeView::*ItemTreeView_sigCheckToggled1)(int) = &ItemTreeView::sigCheckToggled;
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(ItemTreeView_sigCheckToggled1_overloads, sigCheckToggled, 0, 1)
SignalProxy<void(bool)> (ItemTreeView::*ItemTreeView_sigCheckToggled2)(Item*, int) = &ItemTreeView::sigCheckToggled;
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(ItemTreeView_sigCheckToggled2_overloads, sigCheckToggled, 1, 2)

}

namespace cnoid {

void exportPyItemTreeView()
{
    PySignalProxy<void(const ItemList<>&)>("ItemListSignal");
    PySignalProxy<void(Item* item, bool isChecked)>("ItemBoolSignal");
    
    class_<ItemTreeView, ItemTreeView*, bases<View>, boost::noncopyable>("ItemTreeView", no_init)
        .def("instance", &ItemTreeView::instance, return_value_policy<reference_existing_object>()).staticmethod("instance")
        .def("rootItem", ItemTreeView_rootItem)
        .def("showRoot", &ItemTreeView::showRoot)
        .def("selectedItems", &ItemTreeView::selectedItems<Item>)
        .def("selectedItems", ItemTreeView_selectedItems)
        .def("selectedItem", ItemTreeView_selectedItem)
        .def("selectedSubItems", ItemTreeView_selectedSubItems)
        .def("selectedSubItem", ItemTreeView_selectedSubItem)
        .def("isItemSelected", &ItemTreeView::isItemSelected)
        .def("selectItem", &ItemTreeView::selectItem, ItemTreeView_selectItem_overloads())
        .def("selectAllItems", &ItemTreeView::selectAllItems)
        .def("clearSelection", &ItemTreeView::clearSelection)
        .def("checkedItems", ItemTreeView_checkedItems1, ItemTreeView_checkedItems1_overloads())
        .def("checkedItems", ItemTreeView_checkedItems2, ItemTreeView_checkedItems2_overloads())
        .def("isItemChecked", &ItemTreeView::isItemChecked, ItemTreeView_isItemChecked_overloads())
        .def("checkItem", &ItemTreeView::checkItem, ItemTreeView_checkItem_overloads())
        .def("sigSelectionChanged", &ItemTreeView::sigSelectionChanged)
        .def("sigSelectionOrTreeChanged", &ItemTreeView::sigSelectionOrTreeChanged)
        .def("sigCheckToggled", ItemTreeView_sigCheckToggled1, ItemTreeView_sigCheckToggled1_overloads())
        .def("sigCheckToggled", ItemTreeView_sigCheckToggled2, ItemTreeView_sigCheckToggled2_overloads())
        .def("cutSelectedItems", &ItemTreeView::cutSelectedItems)
        ;
}

}
