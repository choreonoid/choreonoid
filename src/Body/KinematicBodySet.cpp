#include "KinematicBodySet.h"
#include <cnoid/CloneMap>

using namespace std;
using namespace cnoid;


KinematicBodySet::KinematicBodySet()
{
    mainBodyPartIndex_ = -1;
    createBodyPartFunc = [](){ return new KinematicBodyPart; };
}


KinematicBodySet::KinematicBodySet(CreateBodyPartFunc createBodyPart)
    : createBodyPartFunc(createBodyPart)
{

}


KinematicBodySet::KinematicBodySet(const KinematicBodySet& org, CloneMap* cloneMap)
    : KinematicBodySet(org.createBodyPartFunc)
{
    bodyParts_.reserve(org.bodyParts_.size());
    for(auto part : org.bodyParts_){
        if(!part){
            bodyParts_.push_back(nullptr);
        } else {
            bodyParts_.push_back(part->clone(cloneMap));
        }
    }
    mainBodyPartIndex_ = org.mainBodyPartIndex_;
}


Referenced* KinematicBodySet::doClone(CloneMap* cloneMap) const
{
    return new KinematicBodySet(*this, cloneMap);
}


KinematicBodyPart* KinematicBodySet::findOrCreateBodyPart(int index)
{
    if(index >= static_cast<int>(bodyParts_.size())){
        bodyParts_.resize(index + 1);
    }
    auto& part = bodyParts_[index];
    if(!part){
        part = createBodyPartFunc();
    }
    return part;
}


void KinematicBodySet::setBodyPart(int index, std::shared_ptr<JointTraverse> jointTraverse)
{
    findOrCreateBodyPart(index)->setJointTraverse(jointTraverse);
}


void KinematicBodySet::setBodyPart(int index, LinkKinematicsKit* kit)
{
    findOrCreateBodyPart(index)->setLinkKinematicsKit(kit);
}

    
void KinematicBodySet::clearBodyPart(int index)
{
    if(index < static_cast<int>(bodyParts_.size())){
        bodyParts_[index] = nullptr;
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
