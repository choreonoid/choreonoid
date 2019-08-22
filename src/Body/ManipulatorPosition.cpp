#include "ManipulatorPosition.h"
#include <cnoid/Body>

using namespace std;
using namespace cnoid;


ManipulatorPosition::ManipulatorPosition(PositionType type)
    : positionType_(type)
{

}
    

ManipulatorPosition::ManipulatorPosition(const ManipulatorPosition& org)
    : positionType_(org.positionType_),
      name_(org.name_)
{

}


ManipulatorIkPosition* ManipulatorPosition::ikPosition()
{
    if(positionType_ == IK){
        return static_cast<ManipulatorIkPosition*>(this);
    }
    return nullptr;
}


ManipulatorFkPosition* ManipulatorPosition::fkPosition()
{
    if(positionType_ == FK){
        return static_cast<ManipulatorFkPosition*>(this);
    }
    return nullptr;
}


ManipulatorIkPosition::ManipulatorIkPosition()
    : ManipulatorPosition(IK)
{
    baseFrame = 0;
    toolFrame = 0;
    configuration = 0;
    phase.fill(0);
}


ManipulatorIkPosition::ManipulatorIkPosition(const ManipulatorIkPosition& org)
    : ManipulatorPosition(org)
{
    translation = org.translation;
    rpy = org.rpy;
    baseFrame = org.baseFrame;
    toolFrame = org.toolFrame;
    configuration = org.configuration;
    phase = org.phase;
}
    
    
ManipulatorIkPosition& ManipulatorIkPosition::operator=(const ManipulatorIkPosition& rhs)
{
    setName(rhs.name());
    
    translation = rhs.translation;
    rpy = rhs.rpy;
    baseFrame = rhs.baseFrame;
    toolFrame = rhs.toolFrame;
    configuration = rhs.configuration;
    phase = rhs.phase;

    return *this;
}
    

ManipulatorPosition* ManipulatorIkPosition::clone()
{
    return new ManipulatorIkPosition(*this);
}


void ManipulatorIkPosition::extract(Body* body)
{

}


bool ManipulatorIkPosition::apply(Body* body) const
{
    return false;
}


ManipulatorFkPosition::ManipulatorFkPosition()
    : ManipulatorPosition(FK)
{
    q.fill(0.0);
}


ManipulatorFkPosition::ManipulatorFkPosition(const ManipulatorFkPosition& org)
    : ManipulatorPosition(org)
{
    q = org.q;
}


ManipulatorFkPosition& ManipulatorFkPosition::operator=(const ManipulatorFkPosition& rhs)
{
    setName(rhs.name());
    q = rhs.q;
    return *this;
}


ManipulatorPosition* ManipulatorFkPosition::clone()
{
    return new ManipulatorFkPosition(*this);
}


void ManipulatorFkPosition::extract(Body* body)
{

}


bool ManipulatorFkPosition::apply(Body* body) const
{
    return false;
}
