/*!
  @author Shin'ichiro Nakaoka
*/

#include "../Signal.h"
#include "../ConnectionSet.h"
#include "LuaSignal.h"

using namespace cnoid;

namespace cnoid {

void exportLuaSignalTypes(sol::table& module)
{
    LuaSignal<void()>("VoidSignal", module);
    LuaSignal<void(bool)>("BoolSignal", module);
    LuaSignal<void(int)>("IntSignal", module);
    LuaSignal<void(double)>("DoubleSignal", module);
    LuaSignal<void(const std::string& str)>("StringSignal", module);

    module.new_usertype<Connection>(
        "Connection",
        "disconnect", &Connection::disconnect,
        "connected", &Connection::connected,
        "block", &Connection::block,
        "unblock", &Connection::unblock);

    module.new_usertype<ScopedConnection>(
        "ScopedConnection",
        "reset", &ScopedConnection::reset,
        "disconnect", &ScopedConnection::disconnect,
        "connected", &ScopedConnection::connected,
        "block", &ScopedConnection::block,
        "unblock", &ScopedConnection::unblock);
    
    module.new_usertype<ConnectionSet>(
        "ConnectionSet",
        "empty", &ConnectionSet::empty,
        "numConnections", &ConnectionSet::numConnections,
        "add", [](ConnectionSet* self, const Connection& c){ self->add(c); },
        //"add", [](ConnectionSet* self, const ConnectionSet& s){ self->add(s); },
        "block", [](ConnectionSet* self){ self->block(); },
        //"block", [](ConnectionSet* self, int index){ self->block(index); },
        "unblock", [](ConnectionSet* self){ self->unblock(); },
        //"unblock", [](ConnectionSet* self, int index){ self->unblock(index); },
        "disconnect", &ConnectionSet::disconnect);

    module.new_usertype<ScopedConnectionSet>(
        "ScopedConnectionSet",
        "empty", &ScopedConnectionSet::empty,
        "numConnections", &ScopedConnectionSet::numConnections,
        "add", &ScopedConnectionSet::add,
        "block", [](ScopedConnectionSet* self){ self->block(); },
        //"block", [](ScopedConnectionSet* self, int index){ self->block(index); },
        "unblock", [](ScopedConnectionSet* self){ self->unblock(); },
        //"unblock", [](ScopedConnectionSet* self, int index){ self->unblock(index); },
        "disconnect", &ScopedConnectionSet::disconnect);
}

}
