#include "MprVariableSet.h"

using namespace std;
using namespace cnoid;


MprVariableSet::MprVariableSet()
{

}


MprVariableSet::MprVariableSet(const MprVariableSet& org)
    : name_(org.name_)
{

}


void MprVariableSet::setName(const std::string& name)
{
    name_ = name;
}
