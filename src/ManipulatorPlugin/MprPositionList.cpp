#include "MprPositionList.h"
#include "MprPosition.h"
#include <cnoid/CloneMap>
#include <cnoid/ValueTree>
#include <fmt/format.h>
#include <unordered_map>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace cnoid {

class MprPositionList::Impl
{
public:
    MprPositionList* self;
    vector<MprPositionPtr> positions;
    unordered_map<GeneralId, MprPositionPtr, GeneralId::Hash> idToPositionMap;
    int idCounter;
    bool isStringIdEnabled;
    Signal<void(int index)> sigPositionAdded;
    Signal<void(int index, MprPosition* position)> sigPositionRemoved;
    Signal<void(int index, int flags)> sigPositionUpdated;

    Impl(MprPositionList* self);
    Impl(MprPositionList* self, const Impl& org);
    bool remove(int index, bool doNotify);
};

}


MprPositionList::MprPositionList()
{
    impl = new Impl(this);
}


MprPositionList::Impl::Impl(MprPositionList* self)
    : self(self)
{
    isStringIdEnabled = true;
    idCounter = 0;    
}


MprPositionList::MprPositionList(const MprPositionList& org, CloneMap* cloneMap)
{
    impl = new MprPositionList::Impl(this, *org.impl);

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


MprPositionList::Impl::Impl(MprPositionList* self, const Impl& org)
    : self(self)
{
    isStringIdEnabled = org.isStringIdEnabled;    
    idCounter = 0;    
}


Referenced* MprPositionList::doClone(CloneMap* cloneMap) const
{
    return new MprPositionList(*this, cloneMap);
}


MprPositionList::~MprPositionList()
{
    clear();
    delete impl;
}
    

void MprPositionList::setStringIdEnabled(bool on)
{
    impl->isStringIdEnabled = on;
}


bool MprPositionList::isStringIdEnabled() const
{
    return impl->isStringIdEnabled;
}


void MprPositionList::clear()
{
    for(auto& position : impl->positions){
        position->ownerPositionList_.reset();
    }
    impl->positions.clear();
    impl->idToPositionMap.clear();
    impl->idCounter = 0;
}


int MprPositionList::numPositions() const
{
    return impl->positions.size();
}


MprPosition* MprPositionList::positionAt(int index) const
{
    return impl->positions[index];
}


int MprPositionList::indexOf(MprPosition* position)
{
    auto pos = std::find(impl->positions.begin(), impl->positions.end(), position);
    if(pos == impl->positions.end()){
        return -1;
    }
    return pos - impl->positions.begin();
}
    

MprPosition* MprPositionList::findPosition(const GeneralId& id) const
{
    auto iter = impl->idToPositionMap.find(id);
    if(iter != impl->idToPositionMap.end()){
        return iter->second;
    }
    
    return nullptr;
}


bool MprPositionList::insert(int index, MprPosition* position)
{
    auto& id = position->id();
    
    if(position->ownerPositionList() || !id.isValid()||
       (!impl->isStringIdEnabled && id.isString()) || findPosition(id)){
        return false;
    }

    position->ownerPositionList_ = this;

    impl->idToPositionMap[id] = position;
    if(index > numPositions()){
        index = numPositions();
    }
    impl->positions.insert(impl->positions.begin() + index, position);

    impl->sigPositionAdded(index);

    return true;
}


bool MprPositionList::replace(int index, MprPosition* position)
{
    auto existing = positionAt(index);
    if(existing->id() != position->id() && findPosition(position->id())){
        return false;
    }
    bool replaced = false;
    if(impl->remove(index, false)){
        if(insert(index, position)){
            replaced = true;
            if(!impl->sigPositionUpdated.empty()){
                impl->sigPositionUpdated(index, MprPosition::ObjectReplaced);
            }
        }
    }
    return replaced;
}


bool MprPositionList::append(MprPosition* position)
{
    return insert(numPositions(), position);
}


void MprPositionList::remove(int index)
{
    impl->remove(index, true);
}


bool MprPositionList::Impl::remove(int index, bool doNotify)
{
    if(index >= positions.size()){
        return false;
    }
    auto position = positions[index];
    position->ownerPositionList_.reset();
    idToPositionMap.erase(position->id());
    positions.erase(positions.begin() + index);
    if(doNotify){
        sigPositionRemoved(index, position);
    }
    return true;
}


SignalProxy<void(int index)> MprPositionList::sigPositionAdded()
{
    return impl->sigPositionAdded;
}


SignalProxy<void(int index, MprPosition* position)> MprPositionList::sigPositionRemoved()
{
    return impl->sigPositionRemoved;
}


SignalProxy<void(int index, int flags)> MprPositionList::sigPositionUpdated()
{
    return impl->sigPositionUpdated;
}


void MprPositionList::notifyPositionUpdate(MprPosition* position, int flags)
{
    if(!impl->sigPositionUpdated.empty()){
        impl->sigPositionUpdated(indexOf(position), flags);
    }
}


bool MprPositionList::resetId(MprPosition* position, const GeneralId& newId)
{
    bool changed = false;

    if(position->ownerPositionList() == this && newId.isValid() &&
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


int MprPositionList::removeUnreferencedPositions
(std::function<bool(MprPosition* position)> isReferenced)
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


void MprPositionList::resetIdCounter()
{
    impl->idCounter = 0;
}


GeneralId MprPositionList::createNextId(int prevId)
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


bool MprPositionList::read(const Mapping& archive)
{
    auto& typeNode = archive.get("type");
    if(typeNode.toString() != "ManipulatorPositionList"){
        typeNode.throwException(
            format(_("{0} cannot be loaded as a manipulator position set"), typeNode.toString()));
    }
        
    auto versionNode = archive.find("format_version");
    if(!*versionNode){
        versionNode = archive.find("formatVersion"); // Old key
    }
    auto version = versionNode->toDouble();
    if(version != 1.0){
        versionNode->throwException(format(_("Format version {0} is not supported."), version));
    }

    clear();

    auto& positionNodes = *archive.findListing("positions");
    if(positionNodes.isValid()){
        for(int i=0; i < positionNodes.size(); ++i){
            auto& node = *positionNodes[i].toMapping();
            auto& typeNode = node["type"];
            auto type = typeNode.toString();
            MprPositionPtr position;
            if(type == "IkPosition"){
                position = new MprIkPosition;
            } else if(type == "FkPosition"){
                position = new MprFkPosition;
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


bool MprPositionList::write(Mapping& archive) const
{
    archive.write("type", "ManipulatorPositionList");
    archive.write("format_version", 1.0);

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
