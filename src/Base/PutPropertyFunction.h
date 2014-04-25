/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_PUT_PROPERTY_FUNCTION_H_INCLUDED
#define CNOID_BASE_PUT_PROPERTY_FUNCTION_H_INCLUDED

#include <cnoid/Selection>
#include <boost/function.hpp>
#include "exportdecl.h"

namespace cnoid {

class PutPropertyFunction
{
public:
        
    virtual ~PutPropertyFunction() { }

    virtual PutPropertyFunction& decimals(int d) = 0;
    virtual PutPropertyFunction& min(double min) = 0;
    virtual PutPropertyFunction& max(double max) = 0;
    virtual PutPropertyFunction& min(int min) = 0;
    virtual PutPropertyFunction& max(int max) = 0;
    virtual PutPropertyFunction& reset() = 0;

    // bool
    virtual void operator()(const std::string& name, bool value) = 0;
    virtual void operator()(const std::string& name, bool value,
                            const boost::function<bool(bool)>& changeFunc) = 0;
    virtual void operator()(const std::string& name, bool value,
                            const boost::function<void(bool)>& changeFunc, bool forceUpdate) = 0;

    // int
    virtual void operator()(const std::string& name, int value) = 0;
    virtual void operator()(const std::string& name, int value,
                            const boost::function<bool(int)>& changeFunc) = 0;
    virtual void operator()(const std::string& name, int value,
                            const boost::function<void(int)>& changeFunc, bool forceUpdate) = 0;

    // double
    virtual void operator()(const std::string& name, double value) = 0;
    virtual void operator()(const std::string& name, double value,
                            const boost::function<bool(double)>& changeFunc) = 0;
    virtual void operator()(const std::string& name, double value,
                            const boost::function<void(double)>& func, bool forceUpdate) = 0;

    // string
    virtual void operator()(const std::string& name, const std::string& value) = 0;
    virtual void operator()(const std::string& name, const std::string& value,
                            const boost::function<bool(const std::string&)>& changeFunc) = 0;
    virtual void operator()(const std::string& name, const std::string& value,
                            const boost::function<void(const std::string&)>& changeFunc, bool forceUpdate) = 0;

    // selection
    virtual void operator()(const std::string& name, const Selection& selection) = 0;
    virtual void operator()(const std::string& name, const Selection& selection,
                            const boost::function<bool(int which)>& changeFunc) = 0;
    virtual void operator()(const std::string& name, const Selection& selection,
                            const boost::function<void(int which)>& changeFunc, bool forceUpdate) = 0;
};


template <class ValueType>
class ChangeProperty
{
    ValueType& variable;
public:
    ChangeProperty(ValueType& variable) : variable(variable) { }
    bool operator()(const ValueType& value){
        variable = value;
        return true;
    }
};

template <>
class ChangeProperty<Selection>
{
    Selection& selection;
public:
    ChangeProperty(Selection& variable) : selection(variable) { }
    bool operator()(int value){
        selection.select(value);
        return true;
    }
};
    
template<class ValueType>
ChangeProperty<ValueType> changeProperty(ValueType& variable) {
    return ChangeProperty<ValueType>(variable);
}
}

#endif
