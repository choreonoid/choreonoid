#include "ManipulatorPositionList.h"
#include "ManipulatorPosition.h"
#include <cnoid/CloneMap>
#include <cnoid/ValueTree>
#include <fmt/format.h>
#include <unordered_map>
#include <regex>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace cnoid {

class ManipulatorPositionList::Impl
{
public:
    ManipulatorPositionList* self;
    vector<ManipulatorPositionPtr> positions;
    unordered_map<GeneralId, ManipulatorPositionPtr, GeneralId::Hash> idToPositionMap;
    int idCounter;
    bool isStringIdEnabled;

    Impl(ManipulatorPositionList* self);
    Impl(ManipulatorPositionList* self, const Impl& org);
};

}


ManipulatorPositionList::ManipulatorPositionList()
{
    impl = new Impl(this);
}


ManipulatorPositionList::Impl::Impl(ManipulatorPositionList* self)
    : self(self)
{
    isStringIdEnabled = true;
}


ManipulatorPositionList::ManipulatorPositionList(const ManipulatorPositionList& org, CloneMap* cloneMap)
{
    impl = new ManipulatorPositionList::Impl(this, *org.impl);

    if(cloneMap){
        for(auto& position : org.impl->positions){
            append(cloneMap->getClone(position));
        }
    } else {
        for(auto& position : org.impl->positions){
            append(position->clone());
        }
    }
}


ManipulatorPositionList::Impl::Impl(ManipulatorPositionList* self, const Impl& org)
    : self(self)
{
    isStringIdEnabled = org.isStringIdEnabled;    
}


Referenced* ManipulatorPositionList::doClone(CloneMap* cloneMap) const
{
    return new ManipulatorPositionList(*this, cloneMap);
}


ManipulatorPositionList::~ManipulatorPositionList()
{
    clear();
    delete impl;
}
    

void ManipulatorPositionList::setStringIdEnabled(bool on)
{
    impl->isStringIdEnabled = on;
}


bool ManipulatorPositionList::isStringIdEnabled() const
{
    return impl->isStringIdEnabled;
}


void ManipulatorPositionList::clear()
{
    for(auto& position : impl->positions){
        position->owner_.reset();
    }
    impl->positions.clear();
    impl->idToPositionMap.clear();
}


int ManipulatorPositionList::numPositions() const
{
    return impl->positions.size();
}


ManipulatorPosition* ManipulatorPositionList::positionAt(int index) const
{
    return impl->positions[index];
}


int ManipulatorPositionList::indexOf(ManipulatorPosition* position)
{
    auto pos = std::find(impl->positions.begin(), impl->positions.end(), position);
    if(pos == impl->positions.end()){
        return -1;
    }
    return pos - impl->positions.begin();
}
    

ManipulatorPosition* ManipulatorPositionList::findPosition(const GeneralId& id) const
{
    auto iter = impl->idToPositionMap.find(id);
    if(iter != impl->idToPositionMap.end()){
        return iter->second;
    }
    
    return nullptr;
}


bool ManipulatorPositionList::insert(int index, ManipulatorPosition* position)
{
    auto& id = position->id();
    
    if(position->owner() || !id.isValid()||
       (!impl->isStringIdEnabled && id.isString()) ||
       ManipulatorPositionList::findPosition(id)){
        return false;
    }

    position->owner_ = this;

    impl->idToPositionMap[id] = position;
    if(index > numPositions()){
        index = numPositions();
    }
    impl->positions.insert(impl->positions.begin() + index, position);

    return true;
}


bool ManipulatorPositionList::append(ManipulatorPosition* position)
{
    return insert(numPositions(), position);
}


void ManipulatorPositionList::removeAt(int index)
{
    if(index >= numPositions()){
        return;
    }
    auto position_ = impl->positions[index];
    position_->owner_.reset();
    impl->idToPositionMap.erase(position_->id());
    impl->positions.erase(impl->positions.begin() + index);
}


bool ManipulatorPositionList::resetId(ManipulatorPosition* position, const GeneralId& newId)
{
    bool changed = false;

    if(position->owner() == this && newId.isValid() &&
       (impl->isStringIdEnabled || newId.isInt())){

        auto& positionMap = impl->idToPositionMap;
        auto iter = positionMap.find(newId);
        if(iter == positionMap.end()){
            positionMap.erase(position->id());
            position->id_ = newId;
            positionMap[newId] = position;
            changed = true;
        }
    }

    return changed;
}


int ManipulatorPositionList::removeUnreferencedPositions
(std::function<bool(ManipulatorPosition* position)> isReferenced)
{
    int numRemoved = 0;
    
    auto iter = impl->positions.begin();
    while(iter != impl->positions.end()){
        auto& position = *iter;
        if(isReferenced(position)){
            ++iter;
        } else {
            impl->idToPositionMap.erase(position->id());
            iter = impl->positions.erase(iter);
            ++numRemoved;
        }
    }

    return numRemoved;
}            


void ManipulatorPositionList::resetIdCounter()
{
    impl->idCounter = 0;
}


GeneralId ManipulatorPositionList::createNextId(int prevId)
{
    if(prevId >= 0){
        impl->idCounter = prevId + 1;
    }
    string name;
    int id;
    while(true){
        id = impl->idCounter++;
        auto iter = impl->idToPositionMap.find(id);
        if(iter == impl->idToPositionMap.end()){
            break;
        }
    }
    return id;
}


bool ManipulatorPositionList::read(const Mapping& archive)
{
    auto& typeNode = archive.get("type");
    if(typeNode.toString() != "ManipulatorPositionList"){
        typeNode.throwException(
            format(_("{0} cannot be loaded as a manipulator position set"), typeNode.toString()));
    }
        
    auto& versionNode = archive.get("formatVersion");
    auto version = versionNode.toDouble();
    if(version != 1.0){
        versionNode.throwException(format(_("Format version {0} is not supported."), version));
    }

    clear();

    auto& positionNodes = *archive.findListing("positions");
    if(positionNodes.isValid()){
        for(int i=0; i < positionNodes.size(); ++i){
            auto& node = *positionNodes[i].toMapping();
            auto& typeNode = node["type"];
            auto type = typeNode.toString();
            ManipulatorPositionPtr position;
            if(type == "IkPosition"){
                position = new ManipulatorIkPosition;
            } else if(type == "FkPosition"){
                position = new ManipulatorFkPosition;
            } else {
                typeNode.throwException(format(_("{0} is not supported"), type));
            }
            if(position){
                if(position->read(node)){
                    append(position);
                }
            }
        }
    }
    
    return true;
}


bool ManipulatorPositionList::write(Mapping& archive) const
{
    archive.write("type", "ManipulatorPositionList");
    archive.write("formatVersion", 1.0);

    if(!impl->positions.empty()){
        Listing& positionNodes = *archive.createListing("positions");
        for(auto& position : impl->positions){
            MappingPtr node = new Mapping;
            if(position->write(*node)){
                positionNodes.append(node);
            }
        }
    }

    return true;
}
