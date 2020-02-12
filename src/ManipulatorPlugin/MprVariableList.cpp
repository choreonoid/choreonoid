#include "MprVariableList.h"
#include <cnoid/CloneMap>
#include <cnoid/ValueTree>
#include <fmt/format.h>
#include <vector>
#include <unordered_map>
#include <functional>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace cnoid {

class MprVariableList::Impl
{
public:
    std::vector<MprVariablePtr> variables;
    unordered_map<GeneralId, MprVariablePtr, GeneralId::Hash> idToVariableMap;
    int idCounter;
    bool isStringIdEnabled;
    Signal<void(MprVariableSet* variableSet, MprVariable* variable)> sigVariableUpdated;
    
    Impl();
    Impl(const Impl& org);
};

}


MprVariableList::MprVariableList()
{
    impl = new Impl;
}


MprVariableList::Impl::Impl()
{
    idCounter = 0;
    isStringIdEnabled = true;
}


MprVariableList::MprVariableList(const MprVariableList& org)
    : MprVariableSet(org)
{
    impl = new Impl(*org.impl);

    impl->variables.reserve(org.impl->variables.size());
    for(auto& variable : org.impl->variables){
        append(new MprVariable(*variable));
    }
}


MprVariableList::MprVariableList(const MprVariableList& org, CloneMap* cloneMap)
    : MprVariableSet(org)
{
    impl = new Impl(*org.impl);

    impl->variables.reserve(org.impl->variables.size());
    for(auto& variable : org.impl->variables){
        append(cloneMap->getClone<MprVariable>(variable));
    }
}


MprVariableList::Impl::Impl(const Impl& org)
{
    idCounter = 0;
    isStringIdEnabled = org.isStringIdEnabled;
}


Referenced* MprVariableList::doClone(CloneMap* cloneMap) const
{
    if(cloneMap){
        return new MprVariableList(*this, cloneMap);
    } else {
        return new MprVariableList(*this);
    }
}


MprVariableList::~MprVariableList()
{
    clear();
    delete impl;
}


void MprVariableList::setStringIdEnabled(bool on)
{
    impl->isStringIdEnabled = on;
}


bool MprVariableList::isStringIdEnabled() const
{
    return impl->isStringIdEnabled;
}


void MprVariableList::clear()
{
    for(auto& variable : impl->variables){
        variable->owner_.reset();
    }
    impl->variables.clear();
    impl->idToVariableMap.clear();
}


int MprVariableList::numVariables() const
{
    return impl->variables.size();
}


int MprVariableList::getNumVariables() const
{
    return numVariables();
}


MprVariable* MprVariableList::variableAt(int index) const
{
    return impl->variables[index];
}


MprVariable* MprVariableList::getVariableAt(int index) const
{
    return variableAt(index);
}


int MprVariableList::indexOf(MprVariable* variable) const
{
    auto pos = std::find(impl->variables.begin(), impl->variables.end(), variable);
    if(pos == impl->variables.end()){
        return -1;
    }
    return pos - impl->variables.begin();
}


MprVariable* MprVariableList::findVariable(const GeneralId& id) const
{
    auto iter = impl->idToVariableMap.find(id);
    if(iter != impl->idToVariableMap.end()){
        return iter->second;
    }
    
    return nullptr;
}


MprVariable* MprVariableList::findOrCreateVariable
(const GeneralId& id, const MprVariable::Value& defaultValue)
{
    auto variable = findVariable(id);

    if(!variable){
        variable = new MprVariable(id);
        append(variable);
    }
    
    return variable;
}



std::vector<MprVariablePtr> MprVariableList::getFindableVariableLists() const
{
    vector<MprVariablePtr> variables;
    vector<MprVariablePtr> namedVariables;

    for(auto& variable : impl->variables){
        auto& id = variable->id();
        if(id.isInt()){
            variables.push_back(variable);
        } else if(id.isString()){
            namedVariables.push_back(variable);
        }
    }

    const int numNumberedVariables = variables.size();
    variables.resize(numNumberedVariables + namedVariables.size());
    std::copy(namedVariables.begin(), namedVariables.end(), variables.begin() + numNumberedVariables);

    return variables;
}


bool MprVariableList::containsVariableSet(const MprVariableSet* variableSet) const
{
    return (variableSet == this);
}


bool MprVariableList::insert(int index, MprVariable* variable)
{
    auto& id = variable->id();

    if(variable->owner() || !id.isValid() ||
       (!impl->isStringIdEnabled && id.isString()) ||
       MprVariableList::findVariable(id)){
        return false;
    }

    variable->owner_ = this;
    impl->idToVariableMap[variable->id()] = variable;
    if(index > numVariables()){
        index = numVariables();
    }
    impl->variables.insert(impl->variables.begin() + index, variable);
    
    return true;
}


bool MprVariableList::append(MprVariable* variable)
{
    return insert(numVariables(), variable);
}


void MprVariableList::removeAt(int index)
{
    if(index >= numVariables()){
        return;
    }
    auto variable_ = impl->variables[index];
    variable_->owner_.reset();
    impl->idToVariableMap.erase(variable_->id());
    impl->variables.erase(impl->variables.begin() + index);
}


bool MprVariableList::resetId(MprVariable* variable, const GeneralId& newId)
{
    bool changed = false;

    if(variable->owner() == this && newId.isValid() &&
       (impl->isStringIdEnabled || newId.isInt())){

        auto& variableMap = impl->idToVariableMap;
        auto iter = variableMap.find(newId);
        if(iter == variableMap.end()){
            variableMap.erase(variable->id());
            variable->id_ = newId;
            variableMap[newId] = variable;
            changed = true;
        }
    }

    return changed;
}


void MprVariableList::resetIdCounter()
{
    impl->idCounter = 0;
}


GeneralId MprVariableList::createNextId(int prevId)
{
    if(prevId >= 0){
        impl->idCounter = prevId + 1;
    }
    string name;
    int id;
    while(true){
        id = impl->idCounter++;
        auto iter = impl->idToVariableMap.find(id);
        if(iter == impl->idToVariableMap.end()){
            break;
        }
    }
    return id;
}


SignalProxy<void(MprVariableSet* variableSet, MprVariable* variable)>
MprVariableList::sigVariableUpdated()
{
    return impl->sigVariableUpdated;
}
    

void MprVariableList::notifyVariableUpdate(MprVariable* variable)
{
    impl->sigVariableUpdated(this, variable);
}


bool MprVariableList::read(const Mapping& archive)
{
    auto& typeNode = archive.get("type");
    if(typeNode.toString() != "ManipulatorVariableList"){
        typeNode.throwException(
            format(_("{0} cannot be loaded as a manipulator variable list"), typeNode.toString()));
    }
        
    auto versionNode = archive.find("format_version");
    if(!*versionNode){
        versionNode = archive.find("formatVersion"); // old
    }
    auto version = versionNode->toDouble();
    if(version != 1.0){
        versionNode->throwException(format(_("Format version {0} is not supported."), version));
    }

    string name;
    if(archive.read("name", name)){
        setName(name);
    }

    clear();

    auto& variableNodes = *archive.findListing("variables");
    if(variableNodes.isValid()){
        for(int i=0; i < variableNodes.size(); ++i){
            auto& node = *variableNodes[i].toMapping();
            MprVariablePtr variable = new MprVariable;
            if(variable->read(node)){
                if(!impl->isStringIdEnabled && variable->id().isString()){
                    node.throwException(
                        format(_("String \"{0}\" is specified as ID, but "
                                 "the string id type is not supported in this system"),
                               variable->id().toString()));
                }
                append(variable);
            }
        }
    }
    
    return true;
}


bool MprVariableList::write(Mapping& archive) const
{
    archive.write("type", "ManipulatorVariableList");
    archive.write("format_version", 1.0);
    if(!name().empty()){
        archive.write("name", name());
    }

    if(!impl->variables.empty()){
        Listing& variableNodes = *archive.createListing("variables");
        for(auto& variable : impl->variables){
            MappingPtr node = new Mapping;
            if(variable->write(*node)){
                variableNodes.append(node);
            }
        }
    }

    return true;
}
