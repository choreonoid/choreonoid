#include "PositionTag.h"
#include <cnoid/EigenArchive>

using namespace std;
using namespace cnoid;


PositionTag::PositionTag()
{
    position_.setIdentity();
}


PositionTag::PositionTag(const Vector3& location)
{
    position_.translation() = location;
}


PositionTag::PositionTag(const PositionTag& org)
    : position_(org.position_)
{

}


bool PositionTag::read(const Mapping* archive)
{
    Vector3 p;
    if(cnoid::read(archive, "translation", p)){
        position_.translation() = p;
        return true;
    }
    return false;
}


void PositionTag::write(Mapping* archive) const
{
    archive->setDoubleFormat("%.9g");
    cnoid::write(archive, "translation", position_.translation());
}
