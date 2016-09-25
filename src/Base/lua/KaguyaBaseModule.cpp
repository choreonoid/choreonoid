/**
   @author Shin'ichiro Nakaoka
*/

#include "../Item.h"
#include "../ExtCommandItem.h"
#include "../MessageView.h"
#include "../TaskView.h"
#include <kaguya.hpp>

using namespace std;
using namespace cnoid;

extern "C" int luaopen_cnoid_Base(lua_State* L)
{
    kaguya::State state(L);
    kaguya::LuaTable module = state.newTable();

    module["View"].setClass(
        kaguya::UserdataMetatable<View>()
        .addFunction("name", &View::name)
        .addFunction("isActive", &View::isActive)
        .addFunction("bringToFront", &View::bringToFront)
        .addStaticFunction("lastFocusView", &View::lastFocusView)
        );
    
    module["MessageView"].setClass(
        kaguya::UserdataMetatable<MessageView, View>()
        .addStaticFunction("instance", &MessageView::instance)
        .addStaticFunction("putln", [](MessageView* self, const char* msg){ self->putln(msg); })
        );

    module["TaskView"].setClass(
        kaguya::UserdataMetatable<TaskView, View>()
        .addStaticFunction("instance", &TaskView::instance)
        );

    module["Item"].setClass(
        kaguya::UserdataMetatable<Item>()
        .addStaticFunction("find", [](const char* path) -> Item* { return Item::find(path); })
        .addFunction("name", &Item::name)
        .addFunction("setName", &Item::setName)
        .addStaticFunction("addChildItem", [](Item* self, Item* item) { return self->addChildItem(item); })
        .addFunction("addSubItem", &Item::addSubItem)
        .addFunction("isSubItem", &Item::isSubItem)
        .addFunction("detachFromParentItem", &Item::detachFromParentItem)
        .addFunction("isTemporal", &Item::isTemporal)
        .addFunction("setTemporal", &Item::setTemporal)
        .addFunction("notifyUpdate", &Item::notifyUpdate)
        );

    module["ExtCommandItem"].setClass(
        kaguya::UserdataMetatable<ExtCommandItem, Item>()
        .setConstructors<ExtCommandItem()>()
        .addStaticFunction("cast", [](Item* item) -> ExtCommandItem* { return dynamic_cast<ExtCommandItem*>(item); })
        .addFunction("setCommand", &ExtCommandItem::setCommand)
        .addFunction("command", &ExtCommandItem::command)
        .addFunction("waitingTimeAfterStarted", &ExtCommandItem::waitingTimeAfterStarted)
        .addFunction("setWaitingTimeAfterStarted", &ExtCommandItem::setWaitingTimeAfterStarted)
        .addFunction("execute", &ExtCommandItem::execute)
        .addFunction("terminate", &ExtCommandItem::terminate)
        );

    return module.push();
}
