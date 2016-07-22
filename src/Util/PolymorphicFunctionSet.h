/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_POLYMORPHIC_FUNCTION_SET_H
#define CNOID_UTIL_POLYMORPHIC_FUNCTION_SET_H

#include <functional>

namespace cnoid {

template <class Object, class Parameter> class PolymorphicFunctionSet
{
    typedef boost::function<void(Parameter p)> Function;
    struct compare {
        bool operator ()(const std::type_info* a, const std::type_info* b) const {
            return a->before(*b);
        }
    };
    typedef std::map<const std::type_info*, Function, compare> FunctionMap;
    FunctionMap functions;

    bool callFuntions(const std::type_info& type, Object* object, Parameter& param){
        FunctionMap::iterator p = functions.find(&type);
        if(p != functions.end()){
            return p->second(param);
        }
        return true;
    }
                    
public:
    template <class Type> void setFunction(FunctionType f) {
        functions[&typeid(Type)] = f;
    }
        
    bool operator()(Object* object, Paramter& param) {
        object->forEachActualType(std::bind(&callFunctions, this, _1, object, std::ref(param)));
    }

};
}
#endif

