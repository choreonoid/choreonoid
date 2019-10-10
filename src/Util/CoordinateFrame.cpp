#include "CoordinateFrame.h"
#include "CoordinateFrameSet.h"
#include "CoordinateFrameContainer.h"
#include <cnoid/CloneMap>
#include <cnoid/EigenArchive>

using namespace std;
using namespace cnoid;

std::string CoordinateFrameId::label() const
{
    if(valueType == Int){
        return std::to_string(intId);
    } else {
        return stringId;
    }
}
    

CoordinateFrame::CoordinateFrame()
{
    T_.setIdentity();
}


CoordinateFrame::CoordinateFrame(const CoordinateFrameId& id)
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


CoordinateFrameSet* CoordinateFrame::ownerFrameSet() const
{
    return ownerFrameSet_.lock();
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
    if(id_.isInt()){
        archive.write("id", id_.toInt());
    } else {
        archive.write("id", id_.toString(), DOUBLE_QUOTED);
    }

    cnoid::write(archive, "translation", Vector3(T_.translation()));
    cnoid::write(archive, "rotation", rpyFromRot(T_.linear()));

    if(!note_.empty()){
        archive.write("note", note_, DOUBLE_QUOTED);
    }
    return true;
}
