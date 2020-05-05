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
    std::vector<CoordinateFramePtr> frames;
    unordered_map<GeneralId, CoordinateFramePtr, GeneralId::Hash> idToFrameMap;
    CoordinateFramePtr identityFrame;
    int idCounter;
    std::string name;
    Signal<void(int index)> sigFrameAdded;
    Signal<void(int index, CoordinateFrame* frame)> sigFrameRemoved;
    Signal<void(int index)> sigFramePositionChanged;
    Signal<void(int index)> sigFrameAttributeChanged;
    
    Impl();
};

}


CoordinateFrameList::CoordinateFrameList()
{
    impl = new Impl;
    frameType_ = Base;
}


CoordinateFrameList::Impl::Impl()
{
    identityFrame = new CoordinateFrame(0);
    idCounter = 1;
}


CoordinateFrameList::CoordinateFrameList(const CoordinateFrameList& org)
{
    impl = new Impl;

    impl->frames.reserve(org.impl->frames.size());
    for(auto& frame : org.impl->frames){
        append(frame->clone());
    }
    impl->name = org.impl->name;

    frameType_ = org.frameType_;
}


Referenced* CoordinateFrameList::doClone(CloneMap* cloneMap) const
{
    return new CoordinateFrameList(*this);
}


const std::string& CoordinateFrameList::name() const
{
    return impl->name;
}


void CoordinateFrameList::setName(const std::string& name)
{
    impl->name = name;
}


CoordinateFrameList::~CoordinateFrameList()
{
    clear();
    delete impl;
}


void CoordinateFrameList::clear()
{
    for(auto& frame : impl->frames){
        frame->ownerFrameList_.reset();
    }
    vector<CoordinateFramePtr> tmpFrames(impl->frames);
    impl->frames.clear();
    impl->idToFrameMap.clear();
    impl->idCounter = 1;

    for(size_t i=0; i < tmpFrames.size(); ++i){
        impl->sigFrameRemoved(i, tmpFrames[i]);
    }
}


int CoordinateFrameList::numFrames() const
{
    return impl->frames.size();
}


CoordinateFrame* CoordinateFrameList::frameAt(int index) const
{
    return impl->frames[index];
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


CoordinateFrame* CoordinateFrameList::getFrame(const GeneralId& id) const
{
    if(auto frame = findFrame(id)){
        return frame;
    }
    return impl->identityFrame;
}


bool CoordinateFrameList::insert(int index, CoordinateFrame* frame)
{
    if(frame->ownerFrameList() || !frame->id().isValid() || frame->id() == 0 ||
       CoordinateFrameList::findFrame(frame->id())){
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

    impl->sigFrameRemoved(index, frame_);
}


SignalProxy<void(int index)> CoordinateFrameList::sigFrameAdded()
{
    return impl->sigFrameAdded;
}


SignalProxy<void(int index, CoordinateFrame* frame)> CoordinateFrameList::sigFrameRemoved()
{
    return impl->sigFrameRemoved;
}


SignalProxy<void(int index)> CoordinateFrameList::sigFramePositionChanged()
{
    return impl->sigFramePositionChanged;
}


SignalProxy<void(int index)> CoordinateFrameList::sigFrameAttributeChanged()
{
    return impl->sigFrameAttributeChanged;
}


void CoordinateFrameList::notifyFramePositionChange(int index)
{
    impl->sigFramePositionChanged(index);
}


void CoordinateFrameList::notifyFrameAttributeChange(int index)
{
    impl->sigFrameAttributeChanged(index);
}


bool CoordinateFrameList::resetId(CoordinateFrame* frame, const GeneralId& newId)
{
    bool changed = false;

    if(frame->ownerFrameList() == this && newId.isValid() && newId != 0){
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

    auto versionNode = archive.find("format_version");
    if(!*versionNode){
        versionNode = archive.find("formatVersion"); // Old key
    }
    auto version = versionNode->toDouble();
    if(version != 1.0){
        versionNode->throwException(format(_("Format version {0} is not supported."), version));
    }

    string symbol;
    if(archive.read("name", symbol)){
        setName(symbol);
    }

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
    if(!name().empty()){
        archive.write("name", name());
    }
    if(frameType_ == Base){
        archive.write("frame_type", "base");
    } else if(frameType_ == Offset){
        archive.write("frame_type", "offset");
    }
}


void CoordinateFrameList::writeFrames(Mapping& archive) const
{
    if(!impl->frames.empty()){
        Listing& frameNodes = *archive.createListing("frames");
        for(auto& frame : impl->frames){
            MappingPtr node = new Mapping;
            if(frame->write(*node)){
                frameNodes.append(node);
            }
        }
    }
}
