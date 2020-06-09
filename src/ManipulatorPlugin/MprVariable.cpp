#include "MprVariable.h"
#include "MprVariableList.h"
#include <cnoid/ValueTree>
#include "gettext.h"

using namespace std;
using namespace cnoid;


MprVariable::MprVariable()
    : MprVariable(0, 0)
{

}


MprVariable::MprVariable(const GeneralId& id, Value value)
    : id_(id),
      value_(value)
{

}


MprVariable::MprVariable(const MprVariable& org)
    : value_(org.value_),
      id_(org.id_),
      note_(org.note_)
{

}


bool MprVariable::resetId(const GeneralId& id)
{
    if(!ownerVariableList_ || !ownerVariableList_.lock()){
        id_ = id;
        return true;
    }
    auto list = ownerVariableList_.lock();
    return list->resetId(this, id);
}


MprVariableList* MprVariable::ownerVariableList() const
{
    return ownerVariableList_.lock();
}


template<class ValueType>
bool MprVariable::setScalarValue(ValueType value)
{
    auto list = ownerVariableList();

    if(!list){
        value_ = value;
        return true;

    } else {
        switch(list->variableType()){

        case MprVariableList::GeneralVariable:
            if(!list->isGeneralVariableValueTypeUnchangeable()){
                value_ = value;
                return true;
            } else {
                switch(valueType()){
                case Int:
                    value_ = static_cast<int>(value);
                    return true;
                case Double:
                    value_ = static_cast<double>(value);
                    return true;
                case Bool:
                    value_ = static_cast<bool>(value);
                    return true;
                case String:
                    return false;
                }
            }
            return false;

        case MprVariableList::IntVariable:
            value_ = static_cast<int>(value);
            return true;

        case MprVariableList::DoubleVariable:
            value_ = static_cast<double>(value);
            return true;

        case MprVariableList::BoolVariable:
            value_ = static_cast<bool>(value);
            return true;

        case MprVariableList::StringVariable:
            return false;
        }
    }

    return false;
}


bool MprVariable::setValue(int value)
{
    return setScalarValue(value);
}


bool MprVariable::setValue(double value)
{
    return setScalarValue(value);
}


bool MprVariable::setValue(bool value)
{
    return setScalarValue(value);
}


bool MprVariable::setValue(const std::string& value)
{
    auto list = ownerVariableList();

    if(!list){
        value_ = value;
        return true;

    } else {
        switch(list->variableType()){

        case MprVariableList::GeneralVariable:
            if(!list->isGeneralVariableValueTypeUnchangeable()){
                value_ = value;
                return true;
            } else {
                switch(valueType()){
                case Int:
                case Double:
                case Bool:
                    return false;
                case String:
                    value_ = value;
                    return true;
                }
            }
            return false;

        case MprVariableList::IntVariable:
        case MprVariableList::DoubleVariable:
        case MprVariableList::BoolVariable:
            return false;

        case MprVariableList::StringVariable:
            value_ = value;
            return true;
        }
    }

    return false;
}


bool MprVariable::setValue(const Value& value)
{
    switch(valueType(value)){
    case Int:    return setValue(intValue(value));
    case Double: return setValue(doubleValue(value));
    case Bool:   return setValue(boolValue(value));
    case String: return setValue(stringValue(value));
    default:
        break;
    }
    return false;
}


static void changeValueToInt(MprVariable::Value& value)
{
    switch(stdx::get_variant_index(value)){
    case MprVariable::Double:
        value = static_cast<int>(stdx::get<double>(value));
        break;
    case MprVariable::Bool:
        value = stdx::get<bool>(value) ? 1 : 0;
        break;
    default:
        value = 0;
        break;
    }
}


static void changeValueToDouble(MprVariable::Value& value)
{
    switch(stdx::get_variant_index(value)){
    case MprVariable::Int:
        value = static_cast<double>(stdx::get<int>(value));
        break;
    case MprVariable::Bool:
        value = stdx::get<bool>(value) ? 1.0 : 0.0;
        break;
    default:
        value = 0.0;
        break;
    }
}


static void changeValueToBool(MprVariable::Value& value)
{
    switch(stdx::get_variant_index(value)){
    case MprVariable::Int:
        value = static_cast<bool>(stdx::get<int>(value));
        break;
    case MprVariable::Double:
        value = static_cast<bool>(stdx::get<double>(value));
        break;
    default:
        value = false;
        break;
    }
}


bool MprVariable::changeValueType(TypeId typeId)
{
    int prevTypeId = valueType();
    if(typeId == prevTypeId){
        return true;
    }
    if(auto list = ownerVariableList()){
        if(list->variableType() == MprVariableList::GeneralVariable &&
           !list->isGeneralVariableValueTypeUnchangeable()){
            switch(typeId){
            case Int:    changeValueToInt(value_);    return true;
            case Double: changeValueToDouble(value_); return true;
            case Bool:   changeValueToBool(value_);   return true;
            case String: value_ = string();           return true;
            default: break;
            }
        }
    }
    return false;
}


std::string MprVariable::toString() const
{
    switch(valueType()){
    case Int:
        return std::to_string(intValue());
    case Double:
        return std::to_string(doubleValue());
    case Bool:
        return boolValue() ? "true" : "false";
    case String:
        return stringValue();
    default:
        return string();
    }
}


void MprVariable::notifyUpdate(int flags)
{
    sigUpdated_(flags);
    if(auto list = ownerVariableList()){
        list->notifyVariableUpdate(this, flags);
    }
}


bool MprVariable::read(const Mapping& archive)
{
    if(id_.read(archive, "id")){

        string type;
        if(!archive.read("value_type", type)){
            archive.read("valueType", type); // old
        }
        if(!type.empty()){
            auto& node = archive["value"];
            if(type == "int"){
                value_ = node.toInt();
            } else if(type == "double"){
                value_ = node.toDouble();
            } else if(type == "bool"){
                value_ = node.toBool();
            } else if(type == "string"){
                value_ = node.toString();
            } else {
                archive.throwException(_("Invalid value type"));
            }

            archive.read("note", note_);
            
            return true;
        }
    }
    return false;
}


bool MprVariable::write(Mapping& archive) const
{
    if(id_.write(archive, "id")){
        int valueType = stdx::get_variant_index(value_);
        switch(valueType){
        case Int:
            archive.write("value_type", "int");
            archive.write("value", stdx::get<int>(value_));
            break;
        case Double:
            archive.write("value_type", "double");
            archive.write("value", stdx::get<double>(value_));
            break;
        case Bool:
            archive.write("value_type", "bool");
            archive.write("value", stdx::get<bool>(value_));
            break;
        case String:
            archive.write("value_type", "string");
            archive.write("value", stdx::get<string>(value_), DOUBLE_QUOTED);
            break;
        default:
            break;
        }
        if(!note_.empty()){
            archive.write("note", note_, DOUBLE_QUOTED);
        }
        return true;
    }
    return false;
}
