#include "LinkCoordinateFrameSet.h"
#include <cnoid/CoordinateFrameList>

using namespace std;
using namespace cnoid;


LinkCoordinateFrameSet::LinkCoordinateFrameSet()
    : MultiCoordinateFrameSet{ new CoordinateFrameList, new CoordinateFrameList, new CoordinateFrameList }
{

}


LinkCoordinateFrameSet::LinkCoordinateFrameSet
(CoordinateFrameSet* worldFrameSet, CoordinateFrameSet* bodyFrameSet, CoordinateFrameSet* endFrameSet)
    : MultiCoordinateFrameSet{ worldFrameSet, bodyFrameSet, endFrameSet }
{

}


LinkCoordinateFrameSet::LinkCoordinateFrameSet(const LinkCoordinateFrameSet& org)
    : MultiCoordinateFrameSet(org)
{

}


LinkCoordinateFrameSet::LinkCoordinateFrameSet(const LinkCoordinateFrameSet& org, CloneMap* cloneMap)
    : MultiCoordinateFrameSet(org, cloneMap)
{

}
    

Referenced* LinkCoordinateFrameSet::doClone(CloneMap* cloneMap) const
{
    if(cloneMap){
        return new LinkCoordinateFrameSet(*this, cloneMap);
    } else {
        return new LinkCoordinateFrameSet(*this);
    }
}
    

LinkCoordinateFrameSet& LinkCoordinateFrameSet::operator=(const LinkCoordinateFrameSet& rhs)
{
    MultiCoordinateFrameSet::operator=(rhs);
    return *this;
}
