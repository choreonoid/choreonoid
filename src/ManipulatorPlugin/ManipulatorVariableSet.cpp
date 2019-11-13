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
