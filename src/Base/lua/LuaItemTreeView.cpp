/*!
 * @author Shin'ichiro Nakaoka
*/

#include "../ItemTreeView.h"
#include "../RootItem.h"
#include <cnoid/LuaUtil>
#include <cnoid/LuaSignal>

namespace cnoid {

void exportLuaItemTreeView(sol::table& module)
{
    //defineLuaSignal<void(const ItemList<>&)>("ItemListSignal", module);
    defineLuaSignal<void(Item* item, bool isChecked)>("ItemBoolSignal", module);
    
    module.new_usertype<ItemTreeView>(
        "ItemTreeView",
        sol::base_classes, sol::bases<View, QWidget, QObject>(),
        "new", sol::no_constructor,
        "instance", &ItemTreeView::instance,
        "rootItem", [](ItemTreeView* self) -> RootItemPtr { return self->rootItem(); },
        "showRoot", &ItemTreeView::showRoot,
        //"selectedItems", &ItemTreeView::selectedItems<Item>,
        "isItemSelected", &ItemTreeView::isItemSelected,
        "selectItem", sol::overload(
            [](ItemTreeView* self, Item* item) { self->selectItem(item); },
            [](ItemTreeView* self, Item* item, bool on) { self->selectItem(item, on); }),
        "selectAllItems", &ItemTreeView::selectAllItems,
        "clearSelection", &ItemTreeView::clearSelection,
        //"checkedItems", ItemTreeView_checkedItems1,
        "isItemChecked", sol::overload(
            [](ItemTreeView* self, Item* item) { return self->isItemChecked(item); },
            [](ItemTreeView* self, Item* item, int id) { return self->isItemChecked(item, id); }),
        "checkItem", sol::overload(
            [](ItemTreeView* self, Item* item) { self->checkItem(item); },
            [](ItemTreeView* self, Item* item, bool on) { self->checkItem(item, on); },
            [](ItemTreeView* self, Item* item, bool on, int id) { self->checkItem(item, on, id); }),
        //"sigSelectionChanged", &ItemTreeView::sigSelectionChanged,
        //"sigSelectionOrTreeChanged", &ItemTreeView::sigSelectionOrTreeChanged,
        "sigCheckToggled", sol::overload(
            [](ItemTreeView* self) { return self->sigCheckToggled(); },
            [](ItemTreeView* self, int id) { return self->sigCheckToggled(id); },
            [](ItemTreeView* self, Item* item) { return self->sigCheckToggled(item); },
            [](ItemTreeView* self, Item* item, int id) { return self->sigCheckToggled(item, id); }),
        "cutSelectedItems", &ItemTreeView::cutSelectedItems);
}

}
