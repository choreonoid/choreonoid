#include "CoordinateFrameList.h"
#include <cnoid/CloneMap>
#include <cnoid/ValueTree>
#include <fmt/format.h>
#include <vector>
#include <unordered_map>
#include <bitset>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace cnoid {

class CoordinateFrameList::Impl
{
public:
    bitset<4> frameTypeBitset;
    std::vector<CoordinateFramePtr> frames;
    unordered_map<GeneralId, CoordinateFramePtr, GeneralId::Hash> idToFrameMap;
    CoordinateFramePtr identityFrame;
    int idCounter;
    std::string name;
    
    Impl();
    void enableFrameType(const std::string& type);
};

}


CoordinateFrameList::CoordinateFrameList()
{
    impl = new Impl;
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
}


Referenced* CoordinateFrameList::doClone(CloneMap* cloneMap) const
{
    return new CoordinateFrameList(*this);
}


void CoordinateFrameList::setFrameTypeEnabled(int type)
{
    impl->frameTypeBitset.set(type);
}


bool CoordinateFrameList::isFrameTypeEnabled(int type) const
{
    return impl->frameTypeBitset[type];
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
    impl->frames.clear();
    impl->idToFrameMap.clear();
    impl->idCounter = 1;
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
    frame_->ownerFrameList_.reset();
    impl->idToFrameMap.erase(frame_->id());
    impl->frames.erase(impl->frames.begin() + index);
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

    string name;
    if(archive.read("name", name)){
        setName(name);
    }

    impl->frameTypeBitset.reset();
    auto type = archive.find("frame_type");
    if(type->isValid()){
        if(type->isString()){
            impl->enableFrameType(type->toString());
        } else if(type->isListing()){
            for(auto& node : *type->toListing()){
                impl->enableFrameType(node->toString());
            }
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


void CoordinateFrameList::Impl::enableFrameType(const std::string& type)
{
    if(type == "global"){
        frameTypeBitset.set(CoordinateFrame::Global);
    } else if(type == "local"){
        frameTypeBitset.set(CoordinateFrame::Local);
    } else if(type == "offset"){
        frameTypeBitset.set(CoordinateFrame::Offset);
    }
}


bool CoordinateFrameList::write(Mapping& archive) const
{
    archive.write("type", "CoordinateFrameList");
    archive.write("format_version", 1.0);
    if(!name().empty()){
        archive.write("name", name());
    }

    ListingPtr types = new Listing;
    if(isFrameTypeEnabled(CoordinateFrame::Global)){
        types->append("global");
    }
    if(isFrameTypeEnabled(CoordinateFrame::Local)){
        types->append("local");
    }
    if(isFrameTypeEnabled(CoordinateFrame::Offset)){
        types->append("offset");
    }
    if(types->size() == 1){
        archive.write("frame_type", types->at(0)->toString());
    } else if(types->size() >= 2){
        types->setFlowStyle();
        archive.insert("frame_type", types);
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
