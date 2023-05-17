#include "KinematicBodySet.h"
#include <cnoid/CloneMap>

using namespace std;
using namespace cnoid;


KinematicBodySet::KinematicBodySet()
{
    numValidBodyParts_ = 0;
    mainBodyPartIndex_ = -1;
}


KinematicBodySet::KinematicBodySet(const KinematicBodySet& org, CloneMap* cloneMap)
{
    numValidBodyParts_ = 0;
    bodyParts_.reserve(org.bodyParts_.size());
    for(auto part : org.bodyParts_){
        if(!part){
            bodyParts_.push_back(nullptr);
        } else {
            bodyParts_.push_back(part->clone(cloneMap));
            ++numValidBodyParts_;
        }
    }
    mainBodyPartIndex_ = org.mainBodyPartIndex_;
}


Referenced* KinematicBodySet::doClone(CloneMap* cloneMap) const
{
    return new KinematicBodySet(*this, cloneMap);
}


void KinematicBodySet::setBodyPart(int index, BodyKinematicsKit* kinematicsKit)
{
    if(index >= static_cast<int>(bodyParts_.size())){
        bodyParts_.resize(index + 1, nullptr);
    }
    auto& part = bodyParts_[index];
    if(!part){
        if(kinematicsKit){
            ++numValidBodyParts_;
        }
    } else {
        if(!kinematicsKit){
            --numValidBodyParts_;
        }
    }
    part = kinematicsKit;
}

    
void KinematicBodySet::clearBodyPart(int index)
{
    if(index < static_cast<int>(bodyParts_.size())){
        auto& part = bodyParts_[index];
        if(part){
            --numValidBodyParts_;
            part = nullptr;
        }
        bool doShrink = true;
        for(size_t i = index + 1; i < bodyParts_.size(); ++i){
            if(bodyParts_[i]){
                doShrink = false;
                break;
            }
        }
        if(doShrink){
            bodyParts_.resize(index);
        }
    }
    if(index == mainBodyPartIndex_){
        mainBodyPartIndex_ = -1;
    }
}


void KinematicBodySet::clear()
{
    bodyParts_.clear();
    numValidBodyParts_ = 0;
    mainBodyPartIndex_ = -1;
}


std::vector<int> KinematicBodySet::validBodyPartIndices() const
{
    std::vector<int> indices;
    for(size_t i=0; i < bodyParts_.size(); ++i){
        if(bodyParts_[i]){
            indices.push_back(i);
        }
    }
    return indices;
}


int KinematicBodySet::indexOf(const BodyKinematicsKit* kit) const
{
    int index = -1;
    if(kit){
        int n = maxIndex() + 1;
        for(int i=0; i < n; ++i){
            if(bodyPart(i) == kit){
                index = i;
                break;
            }
        }
    }
    return index;
}
