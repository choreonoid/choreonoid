/**
   @author Shin'ichiro Nakaoka
*/

#include "../Item.h"
#include "../ExtCommandItem.h"
#include "../MessageView.h"
#include "../TaskView.h"
#include <cnoid/LuaUtil>

using namespace std;
using namespace cnoid;

extern "C" int luaopen_cnoid_Base(lua_State* L)
{
    sol::state_view lua(L);

    lua["require"]("cnoid.Util");

    sol::table module = lua.create_table();
    
    module.new_usertype<View>(
        "View",
        "new", sol::no_constructor,
        "name", &View::name,
        "isActive", &View::isActive,
        "bringToFront", &View::bringToFront,
        "lastFocusView", &View::lastFocusView
        );

    module.new_usertype<MessageView>(
        "MessageView",
        sol::base_classes, sol::bases<View>(),
        "new", sol::no_constructor,
        "instance", &MessageView::instance,
        "put", [](MessageView* self, const char* msg){ self->put(msg); },
        "putln", [](MessageView* self, const char* msg){ self->putln(msg); },
        "notify", [](MessageView* self, const char* msg){ self->notify(msg); },
        "flush", &MessageView::flush,
        "clear", &MessageView::clear
        );

    module.new_usertype<TaskView>(
        "TaskView",
        sol::base_classes, sol::bases<View>(),
        "new", sol::no_constructor,
        "instance", &TaskView::instance
        );

    module.new_usertype<Item>(
        "Item",
        "new", sol::no_constructor,
        "find", [](const char* path) -> ItemPtr { return Item::find(path); },
        "name", &Item::name,
        "setName", &Item::setName,
        "addChildItem", [](Item* self, Item* item) { return self->addChildItem(item); },
        "addSubItem", &Item::addSubItem,
        "isSubItem", &Item::isSubItem,
        "detachFromParentItem", &Item::detachFromParentItem,
        "isTemporal", &Item::isTemporal,
        "setTemporal", &Item::setTemporal,
        "notifyUpdate", &Item::notifyUpdate
        );

    module.new_usertype<ExtCommandItem>(
        "ExtCommandItem",
        sol::base_classes, sol::bases<Item>(),
        "new", sol::factories([]() -> ExtCommandItemPtr { return new ExtCommandItem(); }),
        "cast", [](Item* item) -> ExtCommandItemPtr { return dynamic_cast<ExtCommandItem*>(item); },
        //sol::call_constructor, [](Item* item) -> ExtCommandItemPtr { return dynamic_cast<ExtCommandItem*>(item); },
        "setCommand", &ExtCommandItem::setCommand,
        "command", &ExtCommandItem::command,
        "waitingTimeAfterStarted", &ExtCommandItem::waitingTimeAfterStarted,
        "setWaitingTimeAfterStarted", &ExtCommandItem::setWaitingTimeAfterStarted,
        "execute", &ExtCommandItem::execute,
        "terminate", &ExtCommandItem::terminate
        );

    sol::stack::push(L, module);
    
    return 1;
}
