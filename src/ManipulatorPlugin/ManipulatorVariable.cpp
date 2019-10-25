#include "ManipulatorVariable.h"
#include "ManipulatorVariableSet.h"
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


ManipulatorVariableSet* ManipulatorVariable::ownerVariableSet() const
{
    return ownerVariableSet_.lock();
}


bool ManipulatorVariable::read(const Mapping& archive)
{
    if(id_.read(archive, "id")){
        int type;
        if(archive.read("valueType", type)){
            auto& node = archive["value"];
            switch(type){
            case Bool:
                value_ = node.toBool();
                break;
            case Int:
                value_ = node.toInt();
                break;
            case Double:
                value_ = node.toDouble();
                break;
            case String:
                value_ = node.toString();
                break;
            default:
                archive.throwException(_("Invalid value type"));
                break;
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
        archive.write("valueType", valueType);
        switch(valueType){
        case Bool:
            archive.write("value", stdx::get<bool>(value_));
            break;
        case Int:
            archive.write("value", stdx::get<int>(value_));
            break;
        case Double:
            archive.write("value", stdx::get<double>(value_));
            break;
        case String:
            archive.write("value", stdx::get<string>(value_));
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
