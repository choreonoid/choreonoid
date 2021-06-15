#include "PositionTag.h"
#include <cnoid/EigenArchive>

using namespace std;
using namespace cnoid;


PositionTag::PositionTag()
{
    position_.setIdentity();
    hasAttitude_ = false;
}


PositionTag::PositionTag(const Isometry3& T)
{
    position_ = T;
    hasAttitude_ = true;
}


PositionTag::PositionTag(const Vector3& location)
{
    position_.linear().setIdentity();
    position_.translation() = location;
    hasAttitude_ = false;
}


PositionTag::PositionTag(const PositionTag& org)
    : position_(org.position_),
      hasAttitude_(org.hasAttitude_)
{

}


PositionTag& PositionTag::operator=(const PositionTag& rhs)
{
    position_ = rhs.position_;
    hasAttitude_ = rhs.hasAttitude_;
    return *this;
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


bool PositionTag::write(Mapping* archive) const
{
    archive->setFloatingNumberFormat("%.9g");
    cnoid::write(archive, "translation", position_.translation());
    if(hasAttitude_){
        cnoid::write(archive, "rpy", degree(rpyFromRot(position_.linear())));
    }
    return true;
}
