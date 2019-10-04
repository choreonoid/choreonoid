#include "CoordinateFrameContainer.h"
#include <cnoid/CloneMap>
#include <vector>
#include <unordered_map>

#if __cplusplus <= 201402L
#include <boost/functional/hash.hpp>
#endif


using namespace std;
using namespace cnoid;

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
    if(frame->ownerFrameSet()){
        return false;
    }
    if(auto existing = CoordinateFrameContainer::findFrame(frame->id())){
        return false;
    }
    if(index > numFrames()){
        return false;
    }
    setCoordinateFrameOwner(frame, this);
    impl->idToFrameMap[frame->id()] = frame;
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
            frameMap.erase(iter);
            setCoordinateFrameId(frame, newId);
            frameMap[newId] = frame;
            changed = true;
        }
    }

    return changed;
}
