#include "CoordinateFrame.h"
#include "CoordinateFrameSet.h"
#include <cnoid/CloneMap>
#include <cnoid/EigenArchive>

using namespace std;
using namespace cnoid;


CoordinateFrame::CoordinateFrame()
{
    T_.setIdentity();
    mode_ = Relative;
}


CoordinateFrame::CoordinateFrame(const GeneralId& id)
    : id_(id)
{
    T_.setIdentity();
    mode_ = Relative;
}


CoordinateFrame::CoordinateFrame(const CoordinateFrame& org)
    : T_(org.T_),
      id_(org.id_),
      mode_(org.mode_),
      note_(org.note_)
{

}


CoordinateFrame::CoordinateFrame(const GeneralId& id, CoordinateFrameSet* owner)
    : CoordinateFrame(id)
{
    ownerFrameSet_ = owner;
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
    if(id_.read(archive, "id")){
        Vector3 v;
        if(cnoid::read(archive, "translation", v)){
            T_.translation() = v;
        }
        if(cnoid::read(archive, "rotation", v)){
            T_.linear() = rotFromRpy(v);
        }
        string mode;
        if(archive.read("mode", mode)){
            if(mode == "relative"){
                mode = Relative;
            } else if(mode == "global"){
                mode = Global;
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
        cnoid::write(archive, "translation", Vector3(T_.translation()));
        cnoid::write(archive, "rotation", rpyFromRot(T_.linear()));
        if(mode_ == Relative){
            archive.write("mode", "relative");
        } else if(mode_ == Global){
            archive.write("mode", "global");
        }
        if(!note_.empty()){
            archive.write("note", note_, DOUBLE_QUOTED);
        }
        return true;
    }
    return false;
}
