#include "ManipulatorVariable.h"
#include "ManipulatorVariableList.h"
#include <cnoid/ValueTree>
#include "gettext.h"

using namespace std;
using namespace cnoid;


ManipulatorVariable::ManipulatorVariable()
{

}


ManipulatorVariable::ManipulatorVariable(const GeneralId& id)
    : id_(id)
{

}


ManipulatorVariable::ManipulatorVariable(const ManipulatorVariable& org)
    : value_(org.value_),
      id_(org.id_),
      note_(org.note_)
{

}


Referenced* ManipulatorVariable::doClone(CloneMap*) const
{
    return new ManipulatorVariable(*this);
}


ManipulatorVariable& ManipulatorVariable::operator=(const ManipulatorVariable& rhs)
{
    value_ = rhs.value_;
    note_ = rhs.note_;
    return *this;
}


static void changeValueToInt(ManipulatorVariable::Value& value)
{
    switch(stdx::get_variant_index(value)){
    case ManipulatorVariable::Double:
        value = static_cast<int>(stdx::get<double>(value));
        break;
    case ManipulatorVariable::Bool:
        value = stdx::get<bool>(value) ? 1 : 0;
        break;
    default:
        value = 0;
        break;
    }
}


static void changeValueToDouble(ManipulatorVariable::Value& value)
{
    switch(stdx::get_variant_index(value)){
    case ManipulatorVariable::Int:
        value = static_cast<double>(stdx::get<int>(value));
        break;
    case ManipulatorVariable::Bool:
        value = stdx::get<bool>(value) ? 1.0 : 0.0;
        break;
    default:
        value = 0.0;
        break;
    }
}


static void changeValueToBool(ManipulatorVariable::Value& value)
{
    switch(stdx::get_variant_index(value)){
    case ManipulatorVariable::Int:
        value = static_cast<bool>(stdx::get<int>(value));
        break;
    case ManipulatorVariable::Double:
        value = static_cast<bool>(stdx::get<double>(value));
        break;
    default:
        value = false;
        break;
    }
}


void ManipulatorVariable::changeValueType(int typeId)
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
        

std::string ManipulatorVariable::valueString() const
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


ManipulatorVariableList* ManipulatorVariable::owner() const
{
    return owner_.lock();
}


void ManipulatorVariable::notifyUpdate()
{
    if(auto owner__ = owner()){
        owner__->notifyVariableUpdate(this);
    }
}


bool ManipulatorVariable::read(const Mapping& archive)
{
    if(id_.read(archive, "id")){

        string type;
        if(archive.read("valueType", type)){
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


bool ManipulatorVariable::write(Mapping& archive) const
{
    if(id_.write(archive, "id")){
        int valueType = stdx::get_variant_index(value_);
        switch(valueType){
        case Int:
            archive.write("valueType", "int");
            archive.write("value", stdx::get<int>(value_));
            break;
        case Double:
            archive.write("valueType", "double");
            archive.write("value", stdx::get<double>(value_));
            break;
        case Bool:
            archive.write("valueType", "bool");
            archive.write("value", stdx::get<bool>(value_));
            break;
        case String:
            archive.write("valueType", "string");
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
