/**
   @author Shin'ichiro Nakaoka
*/

#include "../Item.h"
#include "../ExtCommandItem.h"
#include "../MessageView.h"
#include "../TaskView.h"
#include <sol.hpp>
#include <cmath>
#include <iostream>

using namespace std;
using namespace cnoid;

namespace sol {

template <typename T>
struct unique_usertype_traits<cnoid::ref_ptr<T>> {
    typedef T type;
    typedef cnoid::ref_ptr<T> actual_type;
    static const bool value = true;
    static bool is_null(const actual_type& value) {
        return value == nullptr;
    }
    static type* get (const actual_type& p) {
        return p.get();
    }
};

}

namespace {

void stackDump(lua_State* L)
{
    ostream& os = mvout();

    os << "stack: ";
    
    int top = lua_gettop(L);
    for(int i=1; i <= top; ++i){
        int t = lua_type(L, i);
        switch(t){
        case LUA_TSTRING: {
            os << "'" << lua_tostring(L, i) << "'";
            break;
        }
        case LUA_TBOOLEAN: {
            os << (lua_toboolean(L, i) ? "true" : "false");
            break;
        }
        case LUA_TNUMBER: {
            os << lua_tonumber(L, i);
            break;
        }
        default: {
            os << lua_typename(L, t);
            break;
        }
        }
        os << ", ";
    }
    os << endl;
}

}  

extern "C" int luaopen_cnoid_Base(lua_State* L)
{
    sol::state_view lua(L);

    sol::table base = lua.create_table();
    
    base["sin"] = [](double x){ return sin(x); };

    base.new_usertype<View>(
        "View",
        "new", sol::no_constructor,
        "name", &View::name,
        "isActive", &View::isActive,
        "bringToFront", &View::bringToFront,
        "lastFocusView", &View::lastFocusView
        );

    base.new_usertype<MessageView>(
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

    base.new_usertype<TaskView>(
        "TaskView",
        sol::base_classes, sol::bases<View>(),
        "new", sol::no_constructor,
        "instance", &TaskView::instance
        );

    base.new_usertype<Item>(
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

    base.new_usertype<ExtCommandItem>(
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

    sol::stack::push(L, base);
    
    return 1;
}
