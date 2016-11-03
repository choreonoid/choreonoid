/*!
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_LUA_SIGNAL_H
#define CNOID_UTIL_LUA_SIGNAL_H

#include "../Signal.h"
#include <sol.hpp>
#include <boost/type_traits.hpp>

namespace cnoid {

namespace signal_private {

template<typename T> struct lua_function_caller0 {
    sol::function func;
    lua_function_caller0(sol::function func) : func(func) { }
    T operator()() {
        T result;
        try {
            result = func();
        } catch(const sol::error& ex) {
            // put ex.what() by using func.lua_state() and the print function
        }
        return result;
    }
};

template<> struct lua_function_caller0<void> {
    sol::function func;
    lua_function_caller0(sol::function func) : func(func) { }
    void operator()() {
        try {
            func();
        } catch(const sol::error& ex) {

        }
    }
};

template<typename T, typename ARG1> struct lua_function_caller1 {
    sol::function func;
    lua_function_caller1(sol::function func) : func(func) { }
    T operator()(ARG1 arg1) {
        T retval;
        try {
            /*
              The sol::protected_function and sol::protected_function_result types should
              be used instead of sol::function for checking the return value, but the following
              code does not seem to to work correctly in my test. The return value cannot be obtained from the
              protected_function_result even if the lua function returns a value.

              auto result = func(arg1); // func is sol::protected_function
              if(result.valid()){
                  sol::optional<T> value = result;
                  if(value){
                      retval = value.value();
                  } else {
                      // error
                  }
              } else {
                  sol::error err = result;
              }
            */
            retval = func(arg1);

        } catch(const sol::error& ex) {

        }
            
        return retval;
    }
};

template<typename ARG1> struct lua_function_caller1<void, ARG1> {
    sol::function func;
    lua_function_caller1(sol::function func) : func(func) { }
    void operator()(ARG1 arg1) {
        try {
            func(arg1);
        } catch(const sol::error& ex) {

        }
    }
};

template<typename T, typename ARG1, typename ARG2> struct lua_function_caller2 {
    sol::function func;
    lua_function_caller2(sol::function func) : func(func) { }
    T operator()(ARG1 arg1, ARG2 arg2) {
        T result;
        try {
            result = func(arg1, arg2);
        } catch(const sol::error& ex) {

        }
        return result;
    }
};

template<typename ARG1, typename ARG2> struct lua_function_caller2<void, ARG1, ARG2> {
    sol::function func;
    lua_function_caller2(sol::function func) : func(func) { }
    void operator()(ARG1 arg1, ARG2 arg2) {
        try {
            func(arg1, arg2);
        } catch(const sol::error& ex) {

        }
    }
};


template<int Arity, typename Signature, typename Combiner>
class lua_signal_impl;

template<typename Signature, typename Combiner>
class lua_signal_impl<0, Signature, Combiner>
{
    typedef boost::function_traits<Signature> traits;
public:
    typedef lua_function_caller0<typename traits::result_type> caller;
};

template<typename Signature, typename Combiner>
class lua_signal_impl<1, Signature, Combiner>
{
    typedef boost::function_traits<Signature> traits;
public:
    typedef lua_function_caller1<typename traits::result_type,
                                 typename traits::arg1_type> caller;
};

template<typename Signature, typename Combiner>
class lua_signal_impl<2, Signature, Combiner>
{
    typedef boost::function_traits<Signature> traits;
public:
    typedef lua_function_caller2<typename traits::result_type,
                                 typename traits::arg1_type,
                                 typename traits::arg2_type> caller;
};

} // namespace signal_private


template<
    typename Signature, 
    typename Combiner = signal_private::last_value<
        typename signal_private::function_traits<Signature>::result_type>
    >
class LuaSignal : public signal_private::lua_signal_impl<
    (boost::function_traits<Signature>::arity), Signature, Combiner>
{
    typedef Signal<Signature, Combiner> SignalType;
    typedef SignalProxy<Signature, Combiner> SignalProxyType;
    typedef signal_private::lua_signal_impl<(boost::function_traits<Signature>::arity), Signature, Combiner> base_type;
    
    static Connection connect(SignalType& self, sol::function func){
        return self.connect(typename base_type::caller(func));
    }
    static Connection connectProxy(SignalProxyType& self, sol::function func){
        return self.connect(typename base_type::caller(func));
    }
public:
    LuaSignal(const char* name, sol::table& module) {
        module.new_usertype<SignalType>(
            name,
            "new", sol::factories([]() -> std::shared_ptr<SignalType> { return std::make_shared<SignalType>(); }),
            "connect", &LuaSignal::connect);

        module.new_usertype<SignalProxyType>(
            (std::string(name) + "Proxy"),
            "new", sol::factories([](SignalType& signal) { return SignalProxyType(signal); }),
            "connect", &LuaSignal::connectProxy);
    }
};

/*
template<
    typename Signature, 
    typename Combiner = signal_private::last_value<
        typename signal_private::function_traits<Signature>::result_type>
    >
void LuaSignal(const char* name, sol::table& module)
{
    typedef Signal<Signature, Combiner> SignalType;
    typedef SignalProxy<Signature, Combiner> SignalProxyType;
    typedef signal_private::lua_signal_impl<(boost::function_traits<Signature>::arity), Signature, Combiner> signal_impl;
    
    module.new_usertype<SignalType>(
        name,
        "new", sol::factories([]() -> std::shared_ptr<SignalType> { return std::make_shared<SignalType>(); }),
        "connect", [](SignalType& self, sol::function func){
            return self.connect(typename signal_impl::caller(func)); });
    
    module.new_usertype<SignalProxyType>(
        (std::string(name) + "Proxy"),
        "new", sol::factories([](SignalType& signal) { return SignalProxyType(signal); }),
        "connect", [](SignalProxyType& self, sol::function func){
            return self.connect(typename signal_impl::caller(func)); });
}
*/

}

#endif
