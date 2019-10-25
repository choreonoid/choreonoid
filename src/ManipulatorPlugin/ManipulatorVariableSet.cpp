#include "ManipulatorVariableSet.h"

using namespace std;
using namespace cnoid;


ManipulatorVariableSet::ManipulatorVariableSet()
{

}


ManipulatorVariableSet::ManipulatorVariableSet(const ManipulatorVariableSet& org)
    : name_(org.name_)
{

}


void ManipulatorVariableSet::setName(const std::string& name)
{
    name_ = name;
}


void ManipulatorVariableSet::setManipulatorVariableId(ManipulatorVariable* varianble, const GeneralId& id)
{
    varianble->id_ = id;
}


void ManipulatorVariableSet::setManipulatorVariableOwner(ManipulatorVariable* variable, ManipulatorVariableSet* owner)
{
    variable->ownerVariableSet_ = owner;
}
