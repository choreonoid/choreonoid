#include "CoordinateFrameContainer.h"
#include <cnoid/CloneMap>
#include <cnoid/ValueTree>
#include <fmt/format.h>
#include <vector>
#include <unordered_map>

#if __cplusplus <= 201402L
#include <boost/functional/hash.hpp>
#endif

#include "gettext.h"


using namespace std;
using namespace cnoid;
using fmt::format;

namespace cnoid {

class CoordinateFrameContainer::Impl
{
public:
    std::vector<CoordinateFramePtr> frames;
#if __cplusplus > 201402L
    unordered_map<CoordinateFrame::Id, CoordinateFramePtr> idToFrameMap;
#else
    unordered_map<CoordinateFrame::Id, CoordinateFramePtr, boost::hash<CoordinateFrame::Id>> idToFrameMap;
#endif
    CoordinateFramePtr identityFrame;

    int idCounter;
    
    Impl();
};

}


CoordinateFrameContainer::CoordinateFrameContainer()
{
    impl = new Impl;
}


CoordinateFrameContainer::Impl::Impl()
{
    identityFrame = new CoordinateFrame;
    idCounter = 0;
}


CoordinateFrameContainer::CoordinateFrameContainer(const CoordinateFrameContainer& org)
{
    impl = new Impl;

    impl->frames.reserve(org.impl->frames.size());
    for(auto& frame : org.impl->frames){
        append(new CoordinateFrame(*frame));
    }
}


CoordinateFrameContainer::CoordinateFrameContainer(const CoordinateFrameContainer& org, CloneMap* cloneMap)
{
    impl = new Impl;

    impl->frames.reserve(org.impl->frames.size());
    for(auto& frame : org.impl->frames){
        append(cloneMap->getClone<CoordinateFrame>(frame));
    }
}


Referenced* CoordinateFrameContainer::doClone(CloneMap* cloneMap) const
{
    if(cloneMap){
        return new CoordinateFrameContainer(*this, cloneMap);
    } else {
        return new CoordinateFrameContainer(*this);
    }
}


CoordinateFrameContainer::~CoordinateFrameContainer()
{
    clear();
    delete impl;
}


void CoordinateFrameContainer::clear()
{
    for(auto& frame : impl->frames){
        setCoordinateFrameOwner(frame, nullptr);
    }
    impl->frames.clear();
    impl->idToFrameMap.clear();
}


int CoordinateFrameContainer::numFrames() const
{
    return impl->frames.size();
}


int CoordinateFrameContainer::getNumFrames() const
{
    return numFrames();
}


CoordinateFrame* CoordinateFrameContainer::frame(int index) const
{
    return impl->frames[index];
}


CoordinateFrame* CoordinateFrameContainer::getFrame(int index) const
{
    return frame(index);
}


CoordinateFrame* CoordinateFrameContainer::findFrame
(const CoordinateFrame::Id& id, bool returnIdentityFrameIfNotFound) const
{
    auto iter = impl->idToFrameMap.find(id);
    if(iter != impl->idToFrameMap.end()){
        return iter->second;
    }
    return returnIdentityFrameIfNotFound ? impl->identityFrame.get() : nullptr;
}


bool CoordinateFrameContainer::insert(int index, CoordinateFrame* frame)
{
    if(frame->ownerFrameSet() || !frame->hasValidId() ||
       CoordinateFrameContainer::findFrame(frame->id(), false)){
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


bool CoordinateFrameContainer::append(CoordinateFrame* frame)
{
    return insert(numFrames(), frame);
}


void CoordinateFrameContainer::removeAt(int index)
{
    if(index >= numFrames()){
        return;
    }
    auto frame_ = impl->frames[index];
    setCoordinateFrameOwner(frame_, nullptr);
    impl->idToFrameMap.erase(frame_->id());
    impl->frames.erase(impl->frames.begin() + index);
}


bool CoordinateFrameContainer::resetId(CoordinateFrame* frame, const CoordinateFrame::Id& newId)
{
    bool changed = false;

    if(frame->ownerFrameSet() == this){
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


void CoordinateFrameContainer::resetIdCounter()
{
    impl->idCounter = 0;
}


CoordinateFrame::Id CoordinateFrameContainer::createNextId(int prevId)
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


bool CoordinateFrameContainer::read(const Mapping& archive)
{
    auto& typeNode = archive.get("type");
    if(typeNode.toString() != "CoordinateFrameSet"){
        typeNode.throwException(
            format(_("{0} cannot be loaded as a coordinate frame set"), typeNode.toString()));
    }
        
    auto& versionNode = archive.get("formatVersion");
    auto version = versionNode.toDouble();
    if(version != 1.0){
        versionNode.throwException(format(_("Format version {0} is not supported."), version));
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


bool CoordinateFrameContainer::write(Mapping& archive) const
{
    archive.write("type", "CoordinateFrameSet");
    archive.write("formatVersion", 1.0);

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
