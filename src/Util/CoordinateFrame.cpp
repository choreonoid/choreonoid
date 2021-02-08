#include "CoordinateFrame.h"
#include "CoordinateFrameList.h"
#include <cnoid/CloneMap>
#include <cnoid/EigenArchive>

using namespace std;
using namespace cnoid;


CoordinateFrame::CoordinateFrame()
{
    T_.setIdentity();
    mode_ = Local;
}


CoordinateFrame::CoordinateFrame(const GeneralId& id)
    : id_(id)
{
    T_.setIdentity();
    mode_ = Local;
}


CoordinateFrame::CoordinateFrame(const CoordinateFrame& org)
    : T_(org.T_),
      id_(org.id_),
      mode_(org.mode_),
      note_(org.note_)
{

}


CoordinateFrame::CoordinateFrame(const GeneralId& id, CoordinateFrameList* owner)
    : CoordinateFrame(id)
{
    ownerFrameList_ = owner;
}


bool CoordinateFrame::resetId(const GeneralId& id)
{
    if(!ownerFrameList_ || !ownerFrameList_.lock()){
        id_ = id;
        return true;
    }
    auto list = ownerFrameList_.lock();
    return list->resetId(this, id);
}


CoordinateFrameList* CoordinateFrame::ownerFrameList() const
{
    return ownerFrameList_.lock();
}


void CoordinateFrame::notifyUpdate(int flags)
{
    sigUpdated_(flags);
    if(auto frameList = ownerFrameList()){
        frameList->notifyFrameUpdate(this, flags);
    }
}


bool CoordinateFrame::read(const Mapping& archive)
{
    if(id_.read(archive, "id")){
        Vector3 v;
        if(cnoid::read(archive, "translation", v)){
            T_.translation() = v;
        }
        if(cnoid::read(archive, "rpy", v) ||
           cnoid::read(archive, "rotation", v) /* old format */){
            T_.linear() = rotFromRpy(radian(v));
        }
        string symbol;
        if(archive.read("mode", symbol)){
            if(symbol == "global"){
                mode_ = Global;
            } else if(symbol == "local"){
                mode_ = Local;
            }
        }
        archive.read("note", note_);
        return true;
    }
    return false;
}


bool CoordinateFrame::write(Mapping& archive) const
{
    if(id_.write(archive, "id")){
        archive.setFloatingNumberFormat("%.9g");
        cnoid::write(archive, "translation", Vector3(T_.translation()));
        cnoid::write(archive, "rpy", degree(rpyFromRot(T_.linear())));
        if(isGlobal()){
            archive.write("mode", "global");
        }
        if(!note_.empty()){
            archive.write("note", note_, DOUBLE_QUOTED);
        }
        return true;
    }
    return false;
}
