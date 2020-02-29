#include "CoordinateFrameSetSuite.h"
#include "CoordinateFrameList.h"
#include <cnoid/CloneMap>

using namespace std;
using namespace cnoid;


CoordinateFrameSetSuite::CoordinateFrameSetSuite()
{

}


CoordinateFrameSetSuite::CoordinateFrameSetSuite(int numFrameSets)
    : frameSets_(numFrameSets)
{

}


CoordinateFrameSetSuite::CoordinateFrameSetSuite(std::initializer_list<CoordinateFrameSet*> frameSets)
{
    frameSets_.reserve(frameSets.size());
    for(auto& frameSet : frameSets){
        frameSets_.push_back(frameSet);
    }
}


CoordinateFrameSetSuite::CoordinateFrameSetSuite(const CoordinateFrameSetSuite& org)
{
    frameSets_.reserve(org.numFrameSets());
    for(auto& frameSet : org.frameSets_){
        frameSets_.push_back(frameSet->clone());
    }
}


CoordinateFrameSetSuite::CoordinateFrameSetSuite(const CoordinateFrameSetSuite& org, CloneMap* cloneMap)
{
    frameSets_.reserve(org.numFrameSets());
    for(auto& frameSet : org.frameSets_){
        frameSets_.push_back(cloneMap->getClone(frameSet.get()));
    }
}
    
    
Referenced* CoordinateFrameSetSuite::doClone(CloneMap* cloneMap) const
{
    if(cloneMap){
        return new CoordinateFrameSetSuite(*this, cloneMap);
    } else {
        return new CoordinateFrameSetSuite(*this);
    }
}


CoordinateFrameSetSuite& CoordinateFrameSetSuite::operator=(const CoordinateFrameSetSuite& rhs)
{
    const int n = rhs.numFrameSets();
    frameSets_.resize(n);
    for(int i=0; i < n; ++i){
        frameSets_[i] = rhs.frameSets_[i];
    }
    return *this;
}


void CoordinateFrameSetSuite::setFrameSet(int index, CoordinateFrameSet* frameSet)
{
    if(index >= frameSets_.size()){
        frameSets_.resize(index + 1);
    }
    frameSets_[index] = frameSet;
}
