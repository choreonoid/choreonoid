#include "CoordinateFrame.h"
#include "CoordinateFrameList.h"
#include <cnoid/CloneMap>
#include <cnoid/EigenArchive>

using namespace std;
using namespace cnoid;


CoordinateFrame::CoordinateFrame()
{
    T_.setIdentity();
    isGlobal_ = false;
}


CoordinateFrame::CoordinateFrame(const GeneralId& id)
    : id_(id)
{
    T_.setIdentity();
    isGlobal_ = false;
}


CoordinateFrame::CoordinateFrame(const CoordinateFrame& org)
    : T_(org.T_),
      id_(org.id_),
      isGlobal_(org.isGlobal_),
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
        Vector3 v;
        if(cnoid::read(archive, "translation", v)){
            T_.translation() = v;
        }
        if(cnoid::read(archive, "rotation", v)){
            T_.linear() = rotFromRpy(radian(v));
        }
        isGlobal_ = archive.get("is_global", false);
        archive.read("note", note_);
        return true;
    }
    return false;
}


bool CoordinateFrame::write(Mapping& archive) const
{
    if(id_.write(archive, "id")){
        archive.setDoubleFormat("%.9g");
        cnoid::write(archive, "translation", Vector3(T_.translation()));
        cnoid::write(archive, "rotation", degree(rpyFromRot(T_.linear())));
        if(isGlobal_){
            archive.write("is_global", true);
        }
        if(!note_.empty()){
            archive.write("note", note_, DOUBLE_QUOTED);
        }
        return true;
    }
    return false;
}
