#include "ManipulatorVariableSetGroup.h"
#include "ManipulatorVariableList.h"
#include <cnoid/CloneMap>
#include <cnoid/ConnectionSet>
#include <vector>
#include <map>
#include <unordered_set>

using namespace std;
using namespace cnoid;

namespace cnoid {

class ManipulatorVariableSetGroup::Impl
{
public:
    vector<ManipulatorVariableSetPtr> variableSets;
    Signal<void(ManipulatorVariableSet* variableSet, ManipulatorVariable* variable)> sigVariableUpdated;
    ScopedConnectionSet connections;
    Impl();
};

}


ManipulatorVariableSetGroup::ManipulatorVariableSetGroup()
{
    impl = new Impl;
}


ManipulatorVariableSetGroup::Impl::Impl()
{

}


ManipulatorVariableSetGroup::ManipulatorVariableSetGroup(const ManipulatorVariableSetGroup& org)
    : ManipulatorVariableSet(org)
{
    impl = new Impl;

    impl->variableSets.reserve(org.impl->variableSets.size());
    for(auto& variableSet : org.impl->variableSets){
        impl->variableSets.push_back(variableSet->clone());
    }
}


ManipulatorVariableSetGroup::ManipulatorVariableSetGroup(const ManipulatorVariableSetGroup& org, CloneMap* cloneMap)
    : ManipulatorVariableSet(org)
{
    impl = new Impl;

    impl->variableSets.reserve(org.impl->variableSets.size());
    for(auto& variableSet : org.impl->variableSets){
        impl->variableSets.push_back(cloneMap->getClone<ManipulatorVariableSet>(variableSet));
    }
}


Referenced* ManipulatorVariableSetGroup::doClone(CloneMap* cloneMap) const
{
    if(cloneMap){
        return new ManipulatorVariableSetGroup(*this, cloneMap);
    } else {
        return new ManipulatorVariableSetGroup(*this);
    }
}


void ManipulatorVariableSetGroup::clear()
{
    impl->connections.disconnect();
    impl->variableSets.clear();
}


int ManipulatorVariableSetGroup::numVariableSets() const
{
    return impl->variableSets.size();
}


ManipulatorVariableSet* ManipulatorVariableSetGroup::variableSet(int index) const
{
    return impl->variableSets[index];
}


void ManipulatorVariableSetGroup::addVariableSet(ManipulatorVariableSet* variableSet)
{
    impl->variableSets.push_back(variableSet);

    auto impl_ = impl;
    impl->connections.add(
        variableSet->sigVariableUpdated().connect(
            [impl_](ManipulatorVariableSet* variableSet, ManipulatorVariable* variable){
                impl_->sigVariableUpdated(variableSet, variable); }));
}


int ManipulatorVariableSetGroup::getNumVariables() const
{
    int numTotalVariables = 0;
    for(auto& variableSet : impl->variableSets){
        numTotalVariables += variableSet->getNumVariables();
    }
    return numTotalVariables;
}
    

ManipulatorVariable* ManipulatorVariableSetGroup::getVariableAt(int index) const
{
    for(auto& variableSet : impl->variableSets){
        const int n = variableSet->getNumVariables();
        if(index < n){
            return variableSet->getVariableAt(index);
        }
        index -= n;
    }
    return nullptr;
}


ManipulatorVariable* ManipulatorVariableSetGroup::findVariable(const GeneralId& id) const
{
    for(auto& variableSet : impl->variableSets){
        if(auto variable = variableSet->findVariable(id)){
            return variable;
        }
    }
    
    return nullptr;
}


ManipulatorVariable* ManipulatorVariableSetGroup::findOrCreateVariable
(const GeneralId& id, const ManipulatorVariable::Value& defaultValue)
{
    auto variable = findVariable(id);

    if(!variable){
        for(auto& variableSet : impl->variableSets){
            if(variable = variableSet->findOrCreateVariable(id, defaultValue)){
                break;
            }
        }
    }
    
    return variable;
}


std::vector<ManipulatorVariablePtr> ManipulatorVariableSetGroup::getFindableVariableLists() const
{
    // Use the normal map to keep the key order
    typedef map<int, ManipulatorVariable*> NumberedVariableMap;
    NumberedVariableMap numberedVariableMap;
    
    unordered_set<string> namedVariableSet;
    vector<ManipulatorVariable*> namedVariables;

    for(auto& variableSet : impl->variableSets){
        const int n = variableSet->getNumVariables();
        for(int i=0; i < n; ++i){
            auto variable = variableSet->getVariableAt(i);
            auto& id = variable->id();
            if(id.isInt()){
                numberedVariableMap.insert(NumberedVariableMap::value_type(id.toInt(), variable));
            } else if(id.isString()){
                auto inserted = namedVariableSet.insert(id.toString());
                if(inserted.second){
                    namedVariables.push_back(variable);
                }
            }
        }
    }

    vector<ManipulatorVariablePtr> variables;
    variables.reserve(numberedVariableMap.size() + namedVariables.size());
    for(auto& kv : numberedVariableMap){
        variables.push_back(kv.second);
    }
    for(auto& variable : namedVariables){
        variables.push_back(variable);
    }

    return variables;
}


bool ManipulatorVariableSetGroup::containsVariableSet(const ManipulatorVariableSet* variableSet) const
{
    if(variableSet == this){
        return true;
    }
    for(auto& subVariableSet : impl->variableSets){
        if(variableSet == subVariableSet){
            return true;
        }
    }
    return false;
}


SignalProxy<void(ManipulatorVariableSet* variableSet, ManipulatorVariable* variable)>
ManipulatorVariableSetGroup::sigVariableUpdated()
{
    return impl->sigVariableUpdated;
}

