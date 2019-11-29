/**
   @author Shin'ichiro Nakaoka
*/

#include "../RootItem.h"
#include "../ExtCommandItem.h"
#include <cnoid/LuaItemList>

namespace cnoid {

void exportLuaItems(sol::table& module)
{
    module.new_usertype<Item>(
        "Item",
        "new", sol::no_constructor,
        "cast", [](Item* item) -> ItemPtr { return item; },
        "find", [](const char* path) -> ItemPtr { return Item::find(path); },
        "name", &Item::name,
        "setName", &Item::setName,
        "addChildItem", [](Item* self, Item* item) { return self->addChildItem(item); },
        "addSubItem", &Item::addSubItem,
        "isSubItem", &Item::isSubItem,
        "detachFromParentItem", &Item::detachFromParentItem,
        "isTemporal", &Item::isTemporal,
        "setTemporal", &Item::setTemporal,
        "notifyUpdate", &Item::notifyUpdate,

        "descendantItems", [](Item* self, sol::table itemClass, sol::this_state s){
            auto items = self->descendantItems();
            sol::state_view lua(s);
            sol::table matched = lua.create_table();
            int index = 1;
            for(size_t i=0; i < items.size(); ++i){
                sol::object casted = itemClass["cast"](items[i]);
                if(casted != sol::nil){
                    matched[index++] = casted;
                }
            }
            return matched;
        }
        );

    module.new_usertype<RootItem>(
        "RootItem",
        sol::base_classes, sol::bases<Item>(),
        "new", sol::no_constructor,
        "instance", []() -> RootItemPtr { return RootItem::instance(); }
        );
    
    module.new_usertype<ExtCommandItem>(
        "ExtCommandItem",
        sol::base_classes, sol::bases<Item>(),
        "new", sol::factories([]() -> ExtCommandItemPtr { return new ExtCommandItem(); }),
        "cast", [](Item* item) -> ExtCommandItemPtr { return dynamic_cast<ExtCommandItem*>(item); },
        "setCommand", &ExtCommandItem::setCommand,
        "command", &ExtCommandItem::command,
        "waitingTimeAfterStarted", &ExtCommandItem::waitingTimeAfterStarted,
        "setWaitingTimeAfterStarted", &ExtCommandItem::setWaitingTimeAfterStarted,
        "execute", &ExtCommandItem::execute,
        "terminate", &ExtCommandItem::terminate
        );

    LuaItemList<ExtCommandItem>("ExtCommandItem", module);
}

}
