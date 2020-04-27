#include "CoordinateFrame.h"
#include "CoordinateFrameList.h"
#include <cnoid/CloneMap>
#include <cnoid/EigenArchive>

using namespace std;
using namespace cnoid;


CoordinateFrame::CoordinateFrame()
{
    T_.setIdentity();
    frameType_ = Any;
}


CoordinateFrame::CoordinateFrame(const GeneralId& id)
    : id_(id)
{
    T_.setIdentity();
    frameType_ = Any;
}


CoordinateFrame::CoordinateFrame(const CoordinateFrame& org)
    : T_(org.T_),
      id_(org.id_),
      frameType_(org.frameType_),
      note_(org.note_)
{

}


CoordinateFrame::CoordinateFrame(const GeneralId& id, CoordinateFrameList* owner)
    : CoordinateFrame(id)
{
    ownerFrameList_ = owner;
}


CoordinateFrame* CoordinateFrame::clone() const
{
    return new CoordinateFrame(*this);
}


CoordinateFrameList* CoordinateFrame::ownerFrameList() const
{
    return ownerFrameList_.lock();
}


bool CoordinateFrame::read(const Mapping& archive)
{
    if(id_.read(archive, "id")){
        string type;
        if(archive.read("frame_type", type)){
            if(type == "global"){
                frameType_ = Global;
            } else if(type == "local"){
                frameType_ = Local;
            } else if(type == "offset"){
                frameType_ = Offset;
            } else {
                frameType_ = Any;
            }
        }
        Vector3 v;
        if(cnoid::read(archive, "translation", v)){
            T_.translation() = v;
        }
        if(cnoid::read(archive, "rotation", v)){
            T_.linear() = rotFromRpy(radian(v));
        }
        archive.read("note", note_);
        return true;
    }
    return false;
}


bool CoordinateFrame::write(Mapping& archive) const
{
    if(id_.write(archive, "id")){
        if(frameType_ != Any){
            if(frameType_ == Global){
                archive.write("frame_type", "global");
            } else if(frameType_ == Local){
                archive.write("frame_type", "local");
            } else if(frameType_ == Offset){
                archive.write("frame_type", "offset");
            }
        }
        archive.setDoubleFormat("%.9g");
        cnoid::write(archive, "translation", Vector3(T_.translation()));
        cnoid::write(archive, "rotation", degree(rpyFromRot(T_.linear())));
        if(!note_.empty()){
            archive.write("note", note_, DOUBLE_QUOTED);
        }
        return true;
    }
    return false;
}
