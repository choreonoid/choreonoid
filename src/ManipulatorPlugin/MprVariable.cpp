#include "MprVariable.h"
#include "MprVariableList.h"
#include <cnoid/ValueTree>
#include "gettext.h"

using namespace std;
using namespace cnoid;


MprVariable::MprVariable()
{

}


MprVariable::MprVariable(const GeneralId& id)
    : id_(id)
{

}


MprVariable::MprVariable(const MprVariable& org)
    : value_(org.value_),
      id_(org.id_),
      note_(org.note_)
{

}


Referenced* MprVariable::doClone(CloneMap*) const
{
    return new MprVariable(*this);
}


MprVariable& MprVariable::operator=(const MprVariable& rhs)
{
    value_ = rhs.value_;
    note_ = rhs.note_;
    return *this;
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


void MprVariable::changeValueType(int typeId)
{
    int prevTypeId = valueTypeId();
    if(typeId == prevTypeId){
        return;
    }
    switch(typeId){
    case Int:
        changeValueToInt(value_);
        break;
    case Double:
        changeValueToDouble(value_);
        break;
    case Bool:
        changeValueToBool(value_);
        break;
    case String:
        value_ = string();
        break;
    default:
        break;
    }
}
        

std::string MprVariable::valueString() const
{
    switch(valueTypeId()){
    case Int:
        return std::to_string(toInt());
    case Double:
        return std::to_string(toDouble());
    case Bool:
        return toBool() ? "true" : "false";
    case String:
        return toString();
    default:
        return string();
    }
}


MprVariableList* MprVariable::owner() const
{
    return owner_.lock();
}


void MprVariable::notifyUpdate()
{
    if(auto owner__ = owner()){
        owner__->notifyVariableUpdate(this);
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
