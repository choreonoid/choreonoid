/*!
 * @author Shin'ichiro Nakaoka
*/

#include "../ItemTreeView.h"
#include <cnoid/LuaUtil>
#include <cnoid/LuaSignal>
#include "LuaItemList.h"

namespace cnoid {

void exportLuaItemTreeView(sol::table& module)
{
    module.new_usertype<ItemTreeView>(
        "ItemTreeView",
        sol::base_classes, sol::bases<View, QWidget, QObject>(),
        "new", sol::no_constructor,
        "instance", &ItemTreeView::instance
        );

    LuaSignal<void(const ItemList<>&)>("ItemListSignal", module);
    LuaSignal<void(Item* item, bool isChecked)>("ItemBoolSignal", module);
}

}
