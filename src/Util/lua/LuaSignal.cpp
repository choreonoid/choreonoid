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
    typedef Signal<void()> VoidSignal;
    typedef SignalProxy<void()> VoidSignalProxy;
    LuaSignal<void()>("VoidSignal", module);

    typedef Signal<void(bool)> BoolSignal;
    typedef SignalProxy<void(bool)> BoolSignalProxy;
    LuaSignal<void(bool)>("BoolSignal", module);
    
    typedef Signal<void(int)> IntSignal;
    typedef SignalProxy<void(int)> IntSignalProxy;
    LuaSignal<void(int)>("IntSignal", module);
    
    typedef Signal<void(double)> NumberSignal;
    typedef SignalProxy<void(double)> NumberSignalProxy;
    LuaSignal<void(double)>("NumberSignal", module);
    
    typedef Signal<void(const std::string& str)> StringSignal;
    typedef SignalProxy<void(const std::string& str)> StringSignalProxy;
    LuaSignal<void(const std::string& str)>("StringSignal", module);

    module.new_usertype<Connection>(
        "Connection",
        "disconnect", &Connection::disconnect,
        "connected", &Connection::connected,
        "block", &Connection::block,
        "unblock", &Connection::unblock);

    module.new_usertype<ScopedConnection>(
        "ScopedConnection",
        "reset", (void(ScopedConnection::*)()) &ScopedConnection::reset,
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
