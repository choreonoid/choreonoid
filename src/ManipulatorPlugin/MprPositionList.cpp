#include "MprPositionList.h"
#include "MprPosition.h"
#include <cnoid/CloneMap>
#include <cnoid/ValueTree>
#include <cnoid/Format>
#include <unordered_map>
#include "gettext.h"

using namespace std;
using namespace cnoid;

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
    MprPosition* findPosition(const GeneralId& id) const;
    bool insert(int index, MprPosition* position, bool doNotify);
    bool removeAt(int index, bool doNotify);
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
    while(!impl->positions.empty()){
        removeAt(impl->positions.size() - 1);
    }
    resetIdCounter();
}


int MprPositionList::numPositions() const
{
    return impl->positions.size();
}


const MprPosition* MprPositionList::positionAt(int index) const
{
    return impl->positions[index];
}


MprPosition* MprPositionList::positionAt(int index)
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
    return impl->findPosition(id);
}


MprPosition* MprPositionList::Impl::findPosition(const GeneralId& id) const
{
    auto iter = idToPositionMap.find(id);
    if(iter != idToPositionMap.end()){
        return iter->second;
    }
    return nullptr;
}


bool MprPositionList::insert(int index, MprPosition* position)
{
    return impl->insert(index, position, true);
}


bool MprPositionList::Impl::insert(int index, MprPosition* position, bool doNotify)
{
    auto& id = position->id();
    
    if(position->ownerPositionList_ || !id.isValid()||
       (!isStringIdEnabled && id.isString()) || findPosition(id)){
        return false;
    }

    position->ownerPositionList_ = self;

    idToPositionMap[id] = position;

    int size = positions.size();
    if(index > size){
        index = size;
    }
    positions.insert(positions.begin() + index, position);

    if(doNotify){
        sigPositionAdded(index);
    }

    return true;
}


bool MprPositionList::replace(int index, MprPosition* position)
{
    auto existing = positionAt(index);
    if(existing->id() != position->id() && findPosition(position->id())){
        return false;
    }
    bool replaced = false;
    if(impl->removeAt(index, false)){
        if(impl->insert(index, position, false)){
            replaced = true;
            if(impl->sigPositionUpdated.hasConnections()){
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


void MprPositionList::removeAt(int index)
{
    impl->removeAt(index, true);
}


bool MprPositionList::Impl::removeAt(int index, bool doNotify)
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
    if(impl->sigPositionUpdated.hasConnections()){
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


bool MprPositionList::read(const Mapping* archive)
{
    auto& typeNode = archive->get("type");
    if(typeNode.toString() != "ManipulatorPositionList"){
        typeNode.throwException(
            formatR(_("{0} cannot be loaded as a manipulator position set"), typeNode.toString()));
    }
        
    auto versionNode = archive->find("format_version");
    if(!versionNode->isValid()){
        versionNode = archive->find("formatVersion"); // Old key
    }
    auto version = versionNode->toDouble();
    if(version != 1.0){
        versionNode->throwException(formatR(_("Format version {0} is not supported."), version));
    }

    clear();

    auto positionNodes = archive->findListing("positions");
    if(positionNodes->isValid()){
        for(int i=0; i < positionNodes->size(); ++i){
            auto node = positionNodes->at(i)->toMapping();
            auto& typeNode = node->get("type");
            auto type = typeNode.toString();
            MprPositionPtr position;
            if(type == "IkPosition"){
                position = new MprIkPosition;
            } else if(type == "FkPosition"){
                position = new MprFkPosition;
            } else if(type == "CompositePosition"){
                position = new MprCompositePosition;
            } else {
                typeNode.throwException(formatR(_("{0} is not supported"), type));
            }
            if(position){
                if(position->read(node)){
                    if(position->id().isValid()){
                        append(position);
                    } else {
                        node->throwException(_("Invalid position ID"));
                    }
                }
            }
        }
    }
    
    return true;
}


bool MprPositionList::write(Mapping* archive) const
{
    archive->write("type", "ManipulatorPositionList");
    archive->write("format_version", 1.0);

    if(!impl->positions.empty()){
        auto positionNodes = archive->createListing("positions");
        for(auto& position : impl->positions){
            MappingPtr node = new Mapping;
            if(position->write(node)){
                positionNodes->append(node);
            }
        }
    }

    return true;
}
