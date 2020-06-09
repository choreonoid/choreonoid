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
    MprVariableList* self;
    std::vector<MprVariablePtr> variables;
    unordered_map<GeneralId, MprVariablePtr, GeneralId::Hash> idToVariableMap;
    int idCounter;
    Signal<void(int index)> sigVariableAdded;
    Signal<void(int index, MprVariable* variable)> sigVariableRemoved;
    Signal<void(int index, int flags)> sigVariableUpdated;
    
    Impl(MprVariableList* self);
    Impl(MprVariableList* self, const Impl& org);
    bool isIdAvailableAsNewId(const GeneralId& id);
    bool checkType(MprVariable* variable);
    bool insert(int index, MprVariable* variable, bool doTypeCheck);
    bool append(MprVariable* variable, bool doTypeCheck);
};

}


MprVariableList::MprVariableList()
    : MprVariableList(GeneralVariable)
{
    impl = new Impl(this);
}


MprVariableList::MprVariableList(VariableType variableType)
    : variableType_(variableType),
      isGeneralVariableValueTypeUnchangeable_(false),
      isNumberIdEnabled_(true),
      isStringIdEnabled_(false)
{
    impl = new Impl(this);
}


MprVariableList::Impl::Impl(MprVariableList* self)
    : self(self)
{
    idCounter = 0;
}


MprVariableList::MprVariableList(const MprVariableList& org)
    : variableType_(org.variableType_),
      isGeneralVariableValueTypeUnchangeable_(org.isGeneralVariableValueTypeUnchangeable_),
      isNumberIdEnabled_(org.isNumberIdEnabled_),
      isStringIdEnabled_(org.isStringIdEnabled_)
{
    impl = new Impl(this, *org.impl);

    auto& orgVariables = org.impl->variables;
    impl->variables.reserve(orgVariables.size());
    for(auto& variable : orgVariables){
        impl->append(new MprVariable(*variable), false);
    }
}


MprVariableList::MprVariableList(const MprVariableList& org, CloneMap* cloneMap)
    : variableType_(org.variableType_),
      isGeneralVariableValueTypeUnchangeable_(org.isGeneralVariableValueTypeUnchangeable_),
      isNumberIdEnabled_(org.isNumberIdEnabled_),
      isStringIdEnabled_(org.isStringIdEnabled_)
{
    impl = new Impl(this, *org.impl);

    auto& orgVariables = org.impl->variables;
    impl->variables.reserve(orgVariables.size());
    for(auto& variable : orgVariables){
        impl->append(cloneMap->getClone<MprVariable>(variable), false);
    }
}


MprVariableList::Impl::Impl(MprVariableList* self, const Impl& org)
    : self(self)
{
    idCounter = 0;
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


void MprVariableList::setVariableType(VariableType type)
{
    if(numVariables() > 0){
        throw std::invalid_argument(
            "The variable type of MprValiableList cannot be modified when the list has elements");
    }
    variableType_ = type;
}


void MprVariableList::setGeneralVariableValueTypeUnchangeable(bool on)
{
    if(numVariables() > 0){
        throw std::invalid_argument(
            "The flag to make the assinged value type of each variable unchangeable "
            "cannot be modified when the list has elements");
    }
    isGeneralVariableValueTypeUnchangeable_ = on;
}
 

void MprVariableList::setNumberIdEnabled(bool on)
{
    if(numVariables() > 0){
        throw std::invalid_argument(
            "The flag to enable number IDs cannot be modified when the list has elements");
    }
    isNumberIdEnabled_ = on;
}
    

void MprVariableList::setStringIdEnabled(bool on)
{
    if(numVariables() > 0){
        throw std::invalid_argument(
            "The flag to enable string IDs cannot be modified when the list has elements");
    }
    isStringIdEnabled_ = on;
}


void MprVariableList::clear()
{
    while(!impl->variables.empty()){
        removeAt(impl->variables.size() - 1);
    }
    resetIdCounter();
}


int MprVariableList::numVariables() const
{
    return impl->variables.size();
}


MprVariable* MprVariableList::variableAt(int index) const
{
    return impl->variables[index];
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


MprVariable::Value MprVariableList::defaultValue() const
{
    switch(variableType_){
    case GeneralVariable: return 0;
    case IntVariable:     return 0;
    case DoubleVariable:  return 0.0;
    case BoolVariable:    return false;
    case StringVariable:  return string();
    default:
        break;
    }
    return MprVariable::Value();
}


bool MprVariableList::Impl::isIdAvailableAsNewId(const GeneralId& id)
{
    return (id.isValid() &&
            !(!self->isNumberIdEnabled_ && id.isInt()) &&
            !(!self->isStringIdEnabled_ && id.isString()) &&
            !self->findVariable(id));
}


bool MprVariableList::Impl::checkType(MprVariable* variable)
{
    bool isAcceptable = true;
    switch(self->variableType_){
    case IntVariable:
        if(!variable->isInt()){
            isAcceptable = false;
        }
        break;
    case DoubleVariable:
        if(!variable->isDouble()){
            isAcceptable = false;
        }
        break;
    case BoolVariable:
        if(!variable->isBool()){
            isAcceptable = false;
        }
        break;
    case StringVariable:
        if(!variable->isString()){
            isAcceptable = false;
        }
        break;
    default:
        isAcceptable = false;
        break;
    }
    return isAcceptable;
}


bool MprVariableList::insert(int index, MprVariable* variable)
{
    return impl->insert(index, variable, true);
}


bool MprVariableList::Impl::insert(int index, MprVariable* variable, bool doTypeCheck)
{
    auto& id = variable->id();
    if(variable->ownerVariableList_ || !isIdAvailableAsNewId(id)){
        return false;
    }
    if(doTypeCheck){
        if(!checkType(variable)){
            return false;
        }
    }
    
    variable->ownerVariableList_ = self;
    idToVariableMap[id] = variable;
    int numVariables = static_cast<int>(variables.size());
    if(index > numVariables){
        index = numVariables;
    }
    variables.insert(variables.begin() + index, variable);

    sigVariableAdded(index);
    
    return true;
}


bool MprVariableList::append(MprVariable* variable)
{
    return impl->insert(numVariables(), variable, true);
}


bool MprVariableList::Impl::append(MprVariable* variable, bool doTypeCheck)
{
    return insert(variables.size(), variable, doTypeCheck);
}


bool MprVariableList::removeAt(int index)
{
    if(index >= numVariables()){
        return false;
    }
    auto variable = impl->variables[index];
    variable->ownerVariableList_.reset();
    impl->idToVariableMap.erase(variable->id());
    impl->variables.erase(impl->variables.begin() + index);

    impl->sigVariableRemoved(index, variable);

    return true;
}


SignalProxy<void(int index)> MprVariableList::sigVariableAdded()
{
    return impl->sigVariableAdded;
}


SignalProxy<void(int index, MprVariable* variable)> MprVariableList::sigVariableRemoved()
{
    return impl->sigVariableRemoved;
}


SignalProxy<void(int index, int flags)> MprVariableList::sigVariableUpdated()
{
    return impl->sigVariableUpdated;
}
    

void MprVariableList::notifyVariableUpdate(MprVariable* variable, int flags)
{
    if(!impl->sigVariableUpdated.empty()){
        impl->sigVariableUpdated(indexOf(variable), flags);
    }
}


bool MprVariableList::resetId(MprVariable* variable, const GeneralId& newId)
{
    bool changed = false;
    if(variable->ownerVariableList() == this && impl->isIdAvailableAsNewId(newId)){
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

    clear();

    
    auto& vartypeNode = *archive.find("variable_type");
    if(!vartypeNode){
        variableType_ = GeneralVariable;
    } else {
        auto vartype = vartypeNode.toString();
        if(vartype == "general"){
            variableType_ = GeneralVariable;
        } else if(vartype == "integer"){
            variableType_ = IntVariable;
        } else if(vartype == "real"){
            variableType_ = DoubleVariable;
        } else if(vartype == "boolean"){
            variableType_ = BoolVariable;
        } else if(vartype == "string"){
            variableType_ = StringVariable;
        } else {
            vartypeNode.throwException(_("Unknown variable type"));
        }
    }

    archive.read("is_value_type_unchangeable", isGeneralVariableValueTypeUnchangeable_);
    archive.read("is_number_id_enabled", isNumberIdEnabled_);
    archive.read("is_string_id_enabled", isStringIdEnabled_);

    auto& variableNodes = *archive.findListing("variables");
    if(variableNodes.isValid()){
        for(int i=0; i < variableNodes.size(); ++i){
            auto& node = *variableNodes[i].toMapping();
            MprVariablePtr variable = new MprVariable;
            if(variable->read(node)){
                if(!isStringIdEnabled_ && variable->id().isString()){
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

    const char* typeSymbol;
    switch(variableType_){
    case GeneralVariable: typeSymbol = "general"; break;
    case IntVariable:     typeSymbol = "integer"; break;
    case DoubleVariable:  typeSymbol = "real";    break;
    case BoolVariable:    typeSymbol = "boolean"; break;
    case StringVariable:  typeSymbol = "string";  break;
    default:
        return false;
    }
    archive.write("variable_type", typeSymbol);

    if(variableType_ == GeneralVariable && isGeneralVariableValueTypeUnchangeable_){
        archive.write("is_value_type_unchangeable", true);
    }
    if(isNumberIdEnabled_){
        archive.write("is_number_id_enabled", true);
    }
    if(isStringIdEnabled_){
        archive.write("is_string_id_enabled", true);
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
