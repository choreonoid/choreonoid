#include "MultiCoordinateFrameSet.h"
#include "CoordinateFrameList.h"
#include <cnoid/CloneMap>

using namespace std;
using namespace cnoid;


MultiCoordinateFrameSet::MultiCoordinateFrameSet()
{

}


MultiCoordinateFrameSet::MultiCoordinateFrameSet(int numFrameSets)
    : frameSets_(numFrameSets)
{

}


MultiCoordinateFrameSet::MultiCoordinateFrameSet(std::initializer_list<CoordinateFrameSet*> frameSets)
{
    frameSets_.reserve(frameSets.size());
    for(auto& frameSet : frameSets){
        frameSets_.push_back(frameSet);
    }
}


MultiCoordinateFrameSet::MultiCoordinateFrameSet(const MultiCoordinateFrameSet& org)
{
    frameSets_.reserve(org.numFrameSets());
    for(auto& frameSet : org.frameSets_){
        frameSets_.push_back(frameSet->clone());
    }
}


MultiCoordinateFrameSet::MultiCoordinateFrameSet(const MultiCoordinateFrameSet& org, CloneMap* cloneMap)
{
    frameSets_.reserve(org.numFrameSets());
    for(auto& frameSet : org.frameSets_){
        frameSets_.push_back(cloneMap->getClone(frameSet.get()));
    }
}
    
    
Referenced* MultiCoordinateFrameSet::doClone(CloneMap* cloneMap) const
{
    if(cloneMap){
        return new MultiCoordinateFrameSet(*this, cloneMap);
    } else {
        return new MultiCoordinateFrameSet(*this);
    }
}


MultiCoordinateFrameSet& MultiCoordinateFrameSet::operator=(const MultiCoordinateFrameSet& rhs)
{
    const int n = rhs.numFrameSets();
    frameSets_.resize(n);
    for(int i=0; i < n; ++i){
        frameSets_[i] = rhs.frameSets_[i];
    }
    return *this;
}


void MultiCoordinateFrameSet::setFrameSet(int index, CoordinateFrameSet* frameSet)
{
    if(index >= frameSets_.size()){
        frameSets_.resize(index + 1);
    }
    frameSets_[index] = frameSet;
}
