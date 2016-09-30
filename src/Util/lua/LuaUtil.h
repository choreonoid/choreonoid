/*!
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_PYUTIL_H
#define CNOID_UTIL_PYUTIL_H

#include "../Referenced.h"
#include <sol.hpp>
#include <functional>
#include <iosfwd>
#include "exportdecl.h"

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

namespace cnoid {

template<typename T>
T* native(sol::object obj)
{
    if(obj.is<T*>()){
        return obj.as<T*>();
    } else if(obj.is<sol::table>()){
        sol::object cppobj = obj.as<sol::table>()["cppobj"];
        return cppobj.as<T*>();
    }
}

template<typename T>
sol::function makeLuaFunction(lua_State* L, T func)
{
    sol::make_reference(L, sol::as_function(func)).push();
    sol::function luaFunc(L);
    lua_pop(L, 1);
    return luaFunc;
}

CNOID_EXPORT void stackDump(lua_State* L);
CNOID_EXPORT void stackDump(lua_State* L, std::ostream& os);

}

#endif
