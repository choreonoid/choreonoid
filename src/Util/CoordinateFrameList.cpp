#include "CoordinateFrameList.h"
#include <cnoid/CloneMap>
#include <cnoid/ValueTree>
#include <fmt/format.h>
#include <vector>
#include <unordered_map>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace cnoid {

class CoordinateFrameList::Impl
{
public:
    CoordinateFrameList* self;
    std::vector<CoordinateFramePtr> frames;
    unordered_map<GeneralId, CoordinateFramePtr, GeneralId::Hash> idToFrameMap;
    int idCounter;
    std::string name;
    Signal<void(int index)> sigFrameAdded;
    Signal<void(int index, CoordinateFrame* frame)> sigFrameRemoved;
    Signal<void(int index, int flags)> sigFrameUpdated;
    
    Impl(CoordinateFrameList* self);
    void clear(bool doKeepDefaultFrame);
};

}


CoordinateFrameList::CoordinateFrameList()
{
    impl = new Impl(this);
    frameType_ = Base;
}


CoordinateFrameList::Impl::Impl(CoordinateFrameList* self)
    : self(self)
{
    self->hasFirstElementAsDefaultFrame_ = false;
    idCounter = 0;
}


CoordinateFrameList::CoordinateFrameList(const CoordinateFrameList& org)
{
    impl = new Impl(this);

    impl->frames.reserve(org.impl->frames.size());
    for(auto& frame : org.impl->frames){
        append(new CoordinateFrame(*frame));
    }
    hasFirstElementAsDefaultFrame_ = org.hasFirstElementAsDefaultFrame_;
    impl->name = org.impl->name;
    frameType_ = org.frameType_;
}


CoordinateFrameList::~CoordinateFrameList()
{
    impl->clear(false);
    delete impl;
}


Referenced* CoordinateFrameList::doClone(CloneMap* cloneMap) const
{
    return new CoordinateFrameList(*this);
}


CoordinateFrameList& CoordinateFrameList::operator=(const CoordinateFrameList& rhs)
{
    impl->clear(true);
    const int n = rhs.numFrames();
    for(int i=0; i < n; ++i){
        append(new CoordinateFrame(*rhs.frameAt(i)));
    }
    hasFirstElementAsDefaultFrame_ = rhs.hasFirstElementAsDefaultFrame_;
    impl->name = rhs.impl->name;
    frameType_ = rhs.frameType_;
    
    return *this;
}


void CoordinateFrameList::setFirstElementAsDefaultFrame(bool on)
{
    if(on && impl->frames.empty()){
        append(new CoordinateFrame(0));
    }
    hasFirstElementAsDefaultFrame_ = on;
}


bool CoordinateFrameList::isDefaultFrameId(const GeneralId& id) const
{
    if(hasFirstElementAsDefaultFrame_){
        if(impl->frames.front()->id() == id){
            return true;
        }
    }
    return false;
}


void CoordinateFrameList::clear()
{
    impl->clear(true);
}


void CoordinateFrameList::Impl::clear(bool doKeepDefaultFrame)
{
    if(!self->hasFirstElementAsDefaultFrame_){
        doKeepDefaultFrame = false;
    }
    size_t minIndex;
    if(doKeepDefaultFrame){
        minIndex = 1;
    } else {
        minIndex = 0;
    }
    while(frames.size() > minIndex){
        self->removeAt(frames.size() - 1);
    }
    self->resetIdCounter(minIndex);
}


int CoordinateFrameList::numFrames() const
{
    return impl->frames.size();
}


CoordinateFrame* CoordinateFrameList::frameAt(int index) const
{
    if(index < static_cast<int>(impl->frames.size())){
        return impl->frames[index];
    }
    return nullptr;
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
    auto iter = impl->idToFrameMap.find(id);
    if(iter != impl->idToFrameMap.end()){
        return iter->second;
    }
    return nullptr;
}


bool CoordinateFrameList::insert(int index, CoordinateFrame* frame)
{
    if(frame->ownerFrameList_ || !frame->id().isValid() || findFrame(frame->id())){
        return false;
    }

    frame->ownerFrameList_ = this;
    impl->idToFrameMap[frame->id()] = frame;
    if(index > numFrames()){
        index = numFrames();
    }
    impl->frames.insert(impl->frames.begin() + index, frame);

    impl->sigFrameAdded(index);
    
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
    CoordinateFramePtr frame_ = impl->frames[index];
    frame_->ownerFrameList_.reset();
    impl->idToFrameMap.erase(frame_->id());
    impl->frames.erase(impl->frames.begin() + index);

    if(impl->frames.empty()){
        hasFirstElementAsDefaultFrame_ = false;
    }

    impl->sigFrameRemoved(index, frame_);
}


bool CoordinateFrameList::remove(CoordinateFrame* frame)
{
    bool removed = false;
    for(int index = impl->frames.size() - 1; index >= 0; --index){
        if(impl->frames[index] == frame){
            removeAt(index);
            removed = true;
        }
    }
    return removed;
}


SignalProxy<void(int index)> CoordinateFrameList::sigFrameAdded()
{
    return impl->sigFrameAdded;
}


SignalProxy<void(int index, CoordinateFrame* frame)> CoordinateFrameList::sigFrameRemoved()
{
    return impl->sigFrameRemoved;
}


SignalProxy<void(int index, int flags)> CoordinateFrameList::sigFrameUpdated()
{
    return impl->sigFrameUpdated;
}


void CoordinateFrameList::notifyFrameUpdate(CoordinateFrame* frame, int flags)
{
    if(!impl->sigFrameUpdated.empty()){
        impl->sigFrameUpdated(indexOf(frame), flags);
    }
}


bool CoordinateFrameList::resetId(CoordinateFrame* frame, const GeneralId& newId)
{
    bool changed = false;

    if(frame->ownerFrameList() == this && newId.isValid()){
        auto& frameMap = impl->idToFrameMap;
        auto iter = frameMap.find(newId);
        if(iter == frameMap.end()){
            frameMap.erase(frame->id());
            frame->id_ = newId;
            frameMap[newId] = frame;
            changed = true;
        }
    }

    return changed;
}


void CoordinateFrameList::resetIdCounter(int id)
{
    impl->idCounter = id;
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

    auto versionNode = archive.find("format_version");
    if(!*versionNode){
        versionNode = archive.find("formatVersion"); // Old key
    }
    auto version = versionNode->toDouble();
    if(version != 1.0){
        versionNode->throwException(format(_("Format version {0} is not supported."), version));
    }

    string symbol;
    if(archive.read("frame_type", symbol)){
        if(symbol == "base"){
            frameType_ = Base;
        } else if(symbol == "offset"){
            frameType_ = Offset;
        }
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


void CoordinateFrameList::write(Mapping& archive) const
{
    writeHeader(archive);
    writeFrames(archive);
}


void CoordinateFrameList::writeHeader(Mapping& archive) const
{
    archive.write("type", "CoordinateFrameList");
    archive.write("format_version", 1.0);
    if(frameType_ == Base){
        archive.write("frame_type", "base");
    } else if(frameType_ == Offset){
        archive.write("frame_type", "offset");
    }
}


void CoordinateFrameList::writeFrames(Mapping& archive) const
{
    int n = impl->frames.size();
    int index = hasFirstElementAsDefaultFrame_ ? 1 : 0;
    if(index < n){
        Listing& frameNodes = *archive.createListing("frames");
        while(index < n){
            MappingPtr node = new Mapping;
            auto frame = impl->frames[index];
            if(frame->write(*node)){
                frameNodes.append(node);
            }
            ++index;
        }
    }
}
