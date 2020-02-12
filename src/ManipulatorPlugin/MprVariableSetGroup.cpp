#include "MprVariableSetGroup.h"
#include "MprVariableList.h"
#include <cnoid/CloneMap>
#include <cnoid/ConnectionSet>
#include <vector>
#include <map>
#include <unordered_set>

using namespace std;
using namespace cnoid;

namespace cnoid {

class MprVariableSetGroup::Impl
{
public:
    vector<MprVariableSetPtr> variableSets;
    Signal<void(MprVariableSet* variableSet, MprVariable* variable)> sigVariableUpdated;
    ScopedConnectionSet connections;
    Impl();
};

}


MprVariableSetGroup::MprVariableSetGroup()
{
    impl = new Impl;
}


MprVariableSetGroup::Impl::Impl()
{

}


MprVariableSetGroup::MprVariableSetGroup(const MprVariableSetGroup& org)
    : MprVariableSet(org)
{
    impl = new Impl;

    impl->variableSets.reserve(org.impl->variableSets.size());
    for(auto& variableSet : org.impl->variableSets){
        impl->variableSets.push_back(variableSet->clone());
    }
}


MprVariableSetGroup::MprVariableSetGroup(const MprVariableSetGroup& org, CloneMap* cloneMap)
    : MprVariableSet(org)
{
    impl = new Impl;

    impl->variableSets.reserve(org.impl->variableSets.size());
    for(auto& variableSet : org.impl->variableSets){
        impl->variableSets.push_back(cloneMap->getClone<MprVariableSet>(variableSet));
    }
}


Referenced* MprVariableSetGroup::doClone(CloneMap* cloneMap) const
{
    if(cloneMap){
        return new MprVariableSetGroup(*this, cloneMap);
    } else {
        return new MprVariableSetGroup(*this);
    }
}


void MprVariableSetGroup::clear()
{
    impl->connections.disconnect();
    impl->variableSets.clear();
}


int MprVariableSetGroup::numVariableSets() const
{
    return impl->variableSets.size();
}


MprVariableSet* MprVariableSetGroup::variableSet(int index) const
{
    return impl->variableSets[index];
}


void MprVariableSetGroup::addVariableSet(MprVariableSet* variableSet)
{
    impl->variableSets.push_back(variableSet);

    auto impl_ = impl;
    impl->connections.add(
        variableSet->sigVariableUpdated().connect(
            [impl_](MprVariableSet* variableSet, MprVariable* variable){
                impl_->sigVariableUpdated(variableSet, variable); }));
}


int MprVariableSetGroup::getNumVariables() const
{
    int numTotalVariables = 0;
    for(auto& variableSet : impl->variableSets){
        numTotalVariables += variableSet->getNumVariables();
    }
    return numTotalVariables;
}
    

MprVariable* MprVariableSetGroup::getVariableAt(int index) const
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


MprVariable* MprVariableSetGroup::findVariable(const GeneralId& id) const
{
    for(auto& variableSet : impl->variableSets){
        if(auto variable = variableSet->findVariable(id)){
            return variable;
        }
    }
    
    return nullptr;
}


MprVariable* MprVariableSetGroup::findOrCreateVariable
(const GeneralId& id, const MprVariable::Value& defaultValue)
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


std::vector<MprVariablePtr> MprVariableSetGroup::getFindableVariableLists() const
{
    // Use the normal map to keep the key order
    typedef map<int, MprVariable*> NumberedVariableMap;
    NumberedVariableMap numberedVariableMap;
    
    unordered_set<string> namedVariableSet;
    vector<MprVariable*> namedVariables;

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

    vector<MprVariablePtr> variables;
    variables.reserve(numberedVariableMap.size() + namedVariables.size());
    for(auto& kv : numberedVariableMap){
        variables.push_back(kv.second);
    }
    for(auto& variable : namedVariables){
        variables.push_back(variable);
    }

    return variables;
}


bool MprVariableSetGroup::containsVariableSet(const MprVariableSet* variableSet) const
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


SignalProxy<void(MprVariableSet* variableSet, MprVariable* variable)>
MprVariableSetGroup::sigVariableUpdated()
{
    return impl->sigVariableUpdated;
}

