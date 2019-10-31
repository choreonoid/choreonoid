#include "ManipulatorVariableList.h"
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

class ManipulatorVariableList::Impl
{
public:
    std::vector<ManipulatorVariablePtr> variables;
    unordered_map<GeneralId, ManipulatorVariablePtr, GeneralId::Hash> idToVariableMap;
    int idCounter;
    bool isStringIdEnabled;
    Signal<void(ManipulatorVariableSet* variableSet, ManipulatorVariable* variable)> sigVariableUpdated;
    
    Impl();
    Impl(const Impl& org);
};

}


ManipulatorVariableList::ManipulatorVariableList()
{
    impl = new Impl;
}


ManipulatorVariableList::Impl::Impl()
{
    idCounter = 0;
    isStringIdEnabled = true;
}


ManipulatorVariableList::ManipulatorVariableList(const ManipulatorVariableList& org)
    : ManipulatorVariableSet(org)
{
    impl = new Impl(*org.impl);

    impl->variables.reserve(org.impl->variables.size());
    for(auto& variable : org.impl->variables){
        append(new ManipulatorVariable(*variable));
    }
}


ManipulatorVariableList::ManipulatorVariableList(const ManipulatorVariableList& org, CloneMap* cloneMap)
    : ManipulatorVariableSet(org)
{
    impl = new Impl(*org.impl);

    impl->variables.reserve(org.impl->variables.size());
    for(auto& variable : org.impl->variables){
        append(cloneMap->getClone<ManipulatorVariable>(variable));
    }
}


ManipulatorVariableList::Impl::Impl(const Impl& org)
{
    idCounter = 0;
    isStringIdEnabled = org.isStringIdEnabled;
}


Referenced* ManipulatorVariableList::doClone(CloneMap* cloneMap) const
{
    if(cloneMap){
        return new ManipulatorVariableList(*this, cloneMap);
    } else {
        return new ManipulatorVariableList(*this);
    }
}


ManipulatorVariableList::~ManipulatorVariableList()
{
    clear();
    delete impl;
}


void ManipulatorVariableList::setStringIdEnabled(bool on)
{
    impl->isStringIdEnabled = on;
}


bool ManipulatorVariableList::isStringIdEnabled() const
{
    return impl->isStringIdEnabled;
}


void ManipulatorVariableList::clear()
{
    for(auto& variable : impl->variables){
        variable->owner_.reset();
    }
    impl->variables.clear();
    impl->idToVariableMap.clear();
}


int ManipulatorVariableList::numVariables() const
{
    return impl->variables.size();
}


int ManipulatorVariableList::getNumVariables() const
{
    return numVariables();
}


ManipulatorVariable* ManipulatorVariableList::variableAt(int index) const
{
    return impl->variables[index];
}


ManipulatorVariable* ManipulatorVariableList::getVariableAt(int index) const
{
    return variableAt(index);
}


int ManipulatorVariableList::indexOf(ManipulatorVariable* variable) const
{
    auto pos = std::find(impl->variables.begin(), impl->variables.end(), variable);
    if(pos == impl->variables.end()){
        return -1;
    }
    return pos - impl->variables.begin();
}


ManipulatorVariable* ManipulatorVariableList::findVariable(const GeneralId& id) const
{
    auto iter = impl->idToVariableMap.find(id);
    if(iter != impl->idToVariableMap.end()){
        return iter->second;
    }
    
    return nullptr;
}


ManipulatorVariable* ManipulatorVariableList::findOrCreateVariable
(const GeneralId& id, const ManipulatorVariable::Value& defaultValue)
{
    auto variable = findVariable(id);

    if(!variable){
        variable = new ManipulatorVariable(id);
        append(variable);
    }
    
    return variable;
}



std::vector<ManipulatorVariablePtr> ManipulatorVariableList::getFindableVariableLists() const
{
    vector<ManipulatorVariablePtr> variables;
    vector<ManipulatorVariablePtr> namedVariables;

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


bool ManipulatorVariableList::containsVariableSet(const ManipulatorVariableSet* variableSet) const
{
    return (variableSet == this);
}


bool ManipulatorVariableList::insert(int index, ManipulatorVariable* variable)
{
    auto& id = variable->id();

    if(variable->owner() || !id.isValid() ||
       (!impl->isStringIdEnabled && id.isString()) ||
       ManipulatorVariableList::findVariable(id)){
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


bool ManipulatorVariableList::append(ManipulatorVariable* variable)
{
    return insert(numVariables(), variable);
}


void ManipulatorVariableList::removeAt(int index)
{
    if(index >= numVariables()){
        return;
    }
    auto variable_ = impl->variables[index];
    variable_->owner_.reset();
    impl->idToVariableMap.erase(variable_->id());
    impl->variables.erase(impl->variables.begin() + index);
}


bool ManipulatorVariableList::resetId(ManipulatorVariable* variable, const GeneralId& newId)
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


void ManipulatorVariableList::resetIdCounter()
{
    impl->idCounter = 0;
}


GeneralId ManipulatorVariableList::createNextId(int prevId)
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


SignalProxy<void(ManipulatorVariableSet* variableSet, ManipulatorVariable* variable)>
ManipulatorVariableList::sigVariableUpdated()
{
    return impl->sigVariableUpdated;
}
    

void ManipulatorVariableList::notifyVariableUpdate(ManipulatorVariable* variable)
{
    impl->sigVariableUpdated(this, variable);
}


bool ManipulatorVariableList::read(const Mapping& archive)
{
    auto& typeNode = archive.get("type");
    if(typeNode.toString() != "ManipulatorVariableList"){
        typeNode.throwException(
            format(_("{0} cannot be loaded as a manipulator variable list"), typeNode.toString()));
    }
        
    auto& versionNode = archive.get("formatVersion");
    auto version = versionNode.toDouble();
    if(version != 1.0){
        versionNode.throwException(format(_("Format version {0} is not supported."), version));
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
            ManipulatorVariablePtr variable = new ManipulatorVariable;
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


bool ManipulatorVariableList::write(Mapping& archive) const
{
    archive.write("type", "ManipulatorVariableList");
    archive.write("formatVersion", 1.0);
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
