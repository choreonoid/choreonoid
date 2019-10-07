#include "CoordinateFrameSet.h"
#include "CoordinateFrameContainer.h"
#include <cnoid/CloneMap>
#include <cnoid/EigenArchive>

using namespace std;
using namespace cnoid;


CoordinateFrame::CoordinateFrame()
{
    T_.setIdentity();
    id_ = -1; // make invalid
}


CoordinateFrame::CoordinateFrame(const Id& id)
    : id_(id)
{
    T_.setIdentity();
}


CoordinateFrame::CoordinateFrame(const CoordinateFrame& org)
    : T_(org.T_),
      id_(org.id_),
      note_(org.note_)
{

}


Referenced* CoordinateFrame::doClone(CloneMap*) const
{
    return new CoordinateFrame(*this);
}


bool CoordinateFrame::hasValidId() const
{
    int type = stdx::get_variant_index(id_);
    if(type == IntId){
        return stdx::get<int>(id_) >= 0;
    } else {
        return !stdx::get<string>(id_).empty();
    }
}


std::string CoordinateFrame::idLabel() const
{
    int type = stdx::get_variant_index(id_);
    if(type == IntId){
        return std::to_string(stdx::get<int>(id_));
    } else {
        return stdx::get<string>(id_);
    }
}


bool CoordinateFrame::read(const Mapping& archive)
{
    auto idNode = archive.find("id");
    if(idNode->isValid() && idNode->isScalar()){
        auto scalar = static_cast<ScalarNode*>(idNode);
        if(scalar->stringStyle()!= PLAIN_STRING){
            id_ = idNode->toString();
        } else {
            id_ = idNode->toInt();
        }
    } else {
        return false;
    }
    
    Vector3 v;
    if(cnoid::read(archive, "translation", v)){
        T_.translation() = v;
    }
    if(cnoid::read(archive, "rotation", v)){
        T_.linear() = rotFromRpy(v);
    }

    archive.read("note", note_);

    return true;
}


bool CoordinateFrame::write(Mapping& archive) const
{
    int type = stdx::get_variant_index(id_);
    if(type == IntId){
        archive.write("id", stdx::get<int>(id_));
    } else {
        archive.write("id", stdx::get<string>(id_), DOUBLE_QUOTED);
    }

    cnoid::write(archive, "translation", Vector3(T_.translation()));
    cnoid::write(archive, "rotation", rpyFromRot(T_.linear()));

    if(!note_.empty()){
        archive.write("note", note_, DOUBLE_QUOTED);
    }
    return true;
}


void CoordinateFrameSet::setCoordinateFrameId(CoordinateFrame* frame, const CoordinateFrame::Id& id)
{
    frame->id_ = id;
}


void CoordinateFrameSet::setCoordinateFrameOwner(CoordinateFrame* frame, CoordinateFrameSet* owner)
{
    frame->ownerFrameSet_ = owner;
}


CoordinateFrameSetPair::CoordinateFrameSetPair()
{
    for(int i=0; i < 2; ++i){
        frameSets[i] = new CoordinateFrameContainer;
    }
}


CoordinateFrameSetPair::CoordinateFrameSetPair
(CoordinateFrameSet* baseFrames, CoordinateFrameSet* offsetFrames)
{
    frameSets[0] = baseFrames;
    frameSets[1] = offsetFrames;
}


CoordinateFrameSetPair::CoordinateFrameSetPair(const CoordinateFrameSetPair& org)
{
    for(int i=0; i < 2; ++i){
        frameSets[i] = org.frameSets[i]->clone();
    }
}


CoordinateFrameSetPair::CoordinateFrameSetPair(const CoordinateFrameSetPair& org, CloneMap* cloneMap)
{
    for(int i=0; i < 2; ++i){
        frameSets[i] = cloneMap->getClone(org.frameSets[i].get());
    }
}
    
    
Referenced* CoordinateFrameSetPair::doClone(CloneMap* cloneMap) const
{
    if(cloneMap){
        return new CoordinateFrameSetPair(*this, cloneMap);
    } else {
        return new CoordinateFrameSetPair(*this);
    }
}
