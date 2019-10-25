#include "CoordinateFrameList.h"
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

class CoordinateFrameList::Impl
{
public:
    std::vector<CoordinateFramePtr> frames;
    unordered_map<GeneralId, CoordinateFramePtr, GeneralId::Hash> idToFrameMap;
    CoordinateFramePtr identityFrame;
    int idCounter;
    
    Impl();
};

}


CoordinateFrameList::CoordinateFrameList()
{
    impl = new Impl;
}


CoordinateFrameList::Impl::Impl()
{
    identityFrame = new CoordinateFrame(0);
    idCounter = 0;
}


CoordinateFrameList::CoordinateFrameList(const CoordinateFrameList& org)
    : CoordinateFrameSet(org)
{
    impl = new Impl;

    impl->frames.reserve(org.impl->frames.size());
    for(auto& frame : org.impl->frames){
        append(new CoordinateFrame(*frame));
    }
}


CoordinateFrameList::CoordinateFrameList(const CoordinateFrameList& org, CloneMap* cloneMap)
    : CoordinateFrameSet(org)
{
    impl = new Impl;

    impl->frames.reserve(org.impl->frames.size());
    for(auto& frame : org.impl->frames){
        append(cloneMap->getClone<CoordinateFrame>(frame));
    }
}


Referenced* CoordinateFrameList::doClone(CloneMap* cloneMap) const
{
    if(cloneMap){
        return new CoordinateFrameList(*this, cloneMap);
    } else {
        return new CoordinateFrameList(*this);
    }
}


CoordinateFrameList::~CoordinateFrameList()
{
    clear();
    delete impl;
}


void CoordinateFrameList::clear()
{
    for(auto& frame : impl->frames){
        setCoordinateFrameOwner(frame, nullptr);
    }
    impl->frames.clear();
    impl->idToFrameMap.clear();
}


int CoordinateFrameList::numFrames() const
{
    return impl->frames.size();
}


int CoordinateFrameList::getNumFrames() const
{
    return numFrames();
}


CoordinateFrame* CoordinateFrameList::frameAt(int index) const
{
    return impl->frames[index];
}


CoordinateFrame* CoordinateFrameList::getFrameAt(int index) const
{
    return frameAt(index);
}


int CoordinateFrameList::indexOf(CoordinateFrame* frame) const
{
    auto pos = std::find(impl->frames.begin(), impl->frames.end(), frame);
    if(pos == impl->frames.end()){
        return -1;
    }
    return pos - impl->frames.begin();
}


CoordinateFrame* CoordinateFrameList::findFrame(const GeneralId& id) const
{
    if(id == 0){
        return impl->identityFrame;
    }
        
    auto iter = impl->idToFrameMap.find(id);
    if(iter != impl->idToFrameMap.end()){
        return iter->second;
    }
    
    return nullptr;
}


std::vector<CoordinateFramePtr> CoordinateFrameList::getFindableFrameLists() const
{
    vector<CoordinateFramePtr> frames;
    vector<CoordinateFramePtr> namedFrames;

    for(auto& frame : impl->frames){
        auto& id = frame->id();
        if(id.isInt()){
            frames.push_back(frame);
        } else if(id.isString()){
            namedFrames.push_back(frame);
        }
    }

    const int numNumberedFrames = frames.size();
    frames.resize(numNumberedFrames + namedFrames.size());
    std::copy(namedFrames.begin(), namedFrames.end(), frames.begin() + numNumberedFrames);

    return frames;
}


bool CoordinateFrameList::contains(const CoordinateFrameSet* frameSet) const
{
    return (frameSet == this);
}


bool CoordinateFrameList::insert(int index, CoordinateFrame* frame)
{
    if(frame->ownerFrameSet() || !frame->id().isValid() || frame->id() == 0 ||
       CoordinateFrameList::findFrame(frame->id())){
        return false;
    }

    setCoordinateFrameOwner(frame, this);
    impl->idToFrameMap[frame->id()] = frame;
    if(index > numFrames()){
        index = numFrames();
    }
    impl->frames.insert(impl->frames.begin() + index, frame);
    
    return true;
}


bool CoordinateFrameList::append(CoordinateFrame* frame)
{
    return insert(numFrames(), frame);
}


void CoordinateFrameList::removeAt(int index)
{
    if(index >= numFrames()){
        return;
    }
    auto frame_ = impl->frames[index];
    setCoordinateFrameOwner(frame_, nullptr);
    impl->idToFrameMap.erase(frame_->id());
    impl->frames.erase(impl->frames.begin() + index);
}


bool CoordinateFrameList::resetId(CoordinateFrame* frame, const GeneralId& newId)
{
    bool changed = false;

    if(frame->ownerFrameSet() == this && newId.isValid() && newId != 0){
        auto& frameMap = impl->idToFrameMap;
        auto iter = frameMap.find(newId);
        if(iter == frameMap.end()){
            frameMap.erase(frame->id());
            setCoordinateFrameId(frame, newId);
            frameMap[newId] = frame;
            changed = true;
        }
    }

    return changed;
}


void CoordinateFrameList::resetIdCounter()
{
    impl->idCounter = 1;
}


GeneralId CoordinateFrameList::createNextId(int prevId)
{
    if(prevId >= 0){
        impl->idCounter = prevId + 1;
    }
    string name;
    int id;
    while(true){
        id = impl->idCounter++;
        auto iter = impl->idToFrameMap.find(id);
        if(iter == impl->idToFrameMap.end()){
            break;
        }
    }
    return id;
}


bool CoordinateFrameList::read(const Mapping& archive)
{
    auto& typeNode = archive.get("type");
    if(typeNode.toString() != "CoordinateFrameList"){
        typeNode.throwException(
            format(_("{0} cannot be loaded as a coordinate frame list"), typeNode.toString()));
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

    auto& frameNodes = *archive.findListing("frames");
    if(frameNodes.isValid()){
        for(int i=0; i < frameNodes.size(); ++i){
            auto& node = *frameNodes[i].toMapping();
            CoordinateFramePtr frame = new CoordinateFrame;
            if(frame->read(node)){
                append(frame);
            }
        }
    }
    
    return true;
}


bool CoordinateFrameList::write(Mapping& archive) const
{
    archive.write("type", "CoordinateFrameList");
    archive.write("formatVersion", 1.0);
    if(!name().empty()){
        archive.write("name", name());
    }

    if(!impl->frames.empty()){
        Listing& frameNodes = *archive.createListing("frames");
        for(auto& frame : impl->frames){
            MappingPtr node = new Mapping;
            if(frame->write(*node)){
                frameNodes.append(node);
            }
        }
    }

    return true;
}
