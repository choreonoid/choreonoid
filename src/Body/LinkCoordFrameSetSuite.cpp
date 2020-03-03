#include "LinkCoordFrameSetSuite.h"
#include <cnoid/CoordinateFrameList>

using namespace std;
using namespace cnoid;


LinkCoordFrameSetSuite::LinkCoordFrameSetSuite()
    : CoordinateFrameSetSuite{ new CoordinateFrameList, new CoordinateFrameList, new CoordinateFrameList }
{

}


LinkCoordFrameSetSuite::LinkCoordFrameSetSuite
(CoordinateFrameSet* worldFrameSet, CoordinateFrameSet* bodyFrameSet, CoordinateFrameSet* linkFrameSet)
    : CoordinateFrameSetSuite{ worldFrameSet, bodyFrameSet, linkFrameSet }
{

}


LinkCoordFrameSetSuite::LinkCoordFrameSetSuite(const LinkCoordFrameSetSuite& org)
    : CoordinateFrameSetSuite(org)
{

}


LinkCoordFrameSetSuite::LinkCoordFrameSetSuite(const LinkCoordFrameSetSuite& org, CloneMap* cloneMap)
    : CoordinateFrameSetSuite(org, cloneMap)
{

}
    

Referenced* LinkCoordFrameSetSuite::doClone(CloneMap* cloneMap) const
{
    if(cloneMap){
        return new LinkCoordFrameSetSuite(*this, cloneMap);
    } else {
        return new LinkCoordFrameSetSuite(*this);
    }
}
    

LinkCoordFrameSetSuite& LinkCoordFrameSetSuite::operator=(const LinkCoordFrameSetSuite& rhs)
{
    CoordinateFrameSetSuite::operator=(rhs);
    return *this;
}


void LinkCoordFrameSetSuite::resetFrameSets()
{
    int n = numFrameSets();
    for(int i=0; i < n; ++i){
        setFrameSet(0, new CoordinateFrameList);
    }
}
