/*!
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_LUA_UTIL_H
#define CNOID_UTIL_LUA_UTIL_H

#include "../Referenced.h"
#include <sol.hpp>
#include <functional>
#include <iosfwd>
#include <iostream>
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
    return nullptr;
}

template<typename T>
T* native(sol::stack_object obj)
{
    if(obj.is<T*>()){
        return obj.as<T*>();
    } else if(obj.is<sol::stack_table>()){
        sol::object cppobj = obj.as<sol::stack_table>()["cppobj"];
        return cppobj.as<T*>();
    }
    return nullptr;
}

template<typename T>
sol::function makeLuaFunction(lua_State* L, T func)
{
    return sol::make_object(L, sol::as_function(func)).template as<sol::function>();
}

CNOID_EXPORT void stackDump(lua_State* L);
CNOID_EXPORT void stackDump(lua_State* L, std::ostream& os);

}

namespace cnoid {
class Mapping;
class Listing;
}

namespace sol {
template <> struct is_container<cnoid::Mapping> : std::false_type {};
template <> struct is_container<const cnoid::Mapping> : std::false_type {};
template <> struct is_container<cnoid::Listing> : std::false_type {};
template <> struct is_container<const cnoid::Listing> : std::false_type {};
}


#endif
