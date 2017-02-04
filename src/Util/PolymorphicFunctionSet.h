/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_POLYMORPHIC_FUNCTION_SET_H
#define CNOID_UTIL_POLYMORPHIC_FUNCTION_SET_H

#include <functional>
#include <unordered_map>
#include <typeindex>

namespace cnoid {

template <class Processor, class Object> class PolymorphicFunctionSet
{
    typedef std::function<void(Processor& proc, Object* obj)> Function;
    typedef std::unordered_map<std::type_index, Function> FunctionMap;
    FunctionMap functions;

    bool callFunctionForType(Processor& proc, const std::type_info& type, Object* object){
        FunctionMap::iterator p = functions.find(type);
        if(p != functions.end()){
            return p->second(proc, object);
        }
        return true;
    }
                    
public:
    template <class Type> void setFunction(FunctionType f) {
        functions[typeid(Type)] = f;
    }
        
    bool operator()(Processor& porc, Object* object) {
        if(!callFunctionForType(porc, typeid(object), object)){
            return object->callBySuper(
                [&](const std::type_info& type){ return this->callFunctionForType(proc, type, object); });
        }
        return true;
    }
};

}

#endif
