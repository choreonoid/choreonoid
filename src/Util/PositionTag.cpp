#include "PositionTag.h"
#include <cnoid/EigenArchive>

using namespace std;
using namespace cnoid;


PositionTag::PositionTag()
{
    position_.setIdentity();
    hasAttitude_ = false;
}


PositionTag::PositionTag(const Position& T)
{
    position_ = T;
}


PositionTag::PositionTag(const Vector3& location)
{
    position_.translation() = location;
}


PositionTag::PositionTag(const PositionTag& org)
    : position_(org.position_),
      hasAttitude_(org.hasAttitude_)
{

}


bool PositionTag::read(const Mapping* archive)
{
    bool isValid = false;
    Vector3 v;
    if(cnoid::read(archive, "translation", v)){
        position_.translation() = v;
        isValid = true;
    }
    if(cnoid::read(archive, "rpy", v)){
        position_.linear() = rotFromRpy(radian(v));
        hasAttitude_ = true;
    } else {
        hasAttitude_ = false;
    }
    return isValid;
}


void PositionTag::write(Mapping* archive) const
{
    archive->setDoubleFormat("%.9g");
    cnoid::write(archive, "translation", position_.translation());
    if(hasAttitude_){
        cnoid::write(archive, "rpy", degree(rpyFromRot(position_.linear())));
    }
}
