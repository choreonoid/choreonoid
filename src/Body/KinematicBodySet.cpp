#include "KinematicBodySet.h"
#include <cnoid/CloneMap>

using namespace std;
using namespace cnoid;


KinematicBodySet::KinematicBodySet()
{
    mainBodyPartIndex_ = -1;
    
    createBodyPartFunc =
        [](){ return new BodyPart; };
    copyBodyPartFunc =
        [this](BodyPart* newBodyPart, BodyPart* orgBodyPart, CloneMap* cloneMap){
            copyBodyPart(newBodyPart, orgBodyPart, cloneMap);
        };
}


KinematicBodySet::KinematicBodySet(CreateBodyPartFunc createBodyPart, CopyBodyPartFunc copyBodyPart)
    : createBodyPartFunc(createBodyPart),
      copyBodyPartFunc(copyBodyPart)
{

}


KinematicBodySet::KinematicBodySet(const KinematicBodySet& org, CloneMap* cloneMap)
    : KinematicBodySet(org.createBodyPartFunc, org.copyBodyPartFunc)
{
    bodyParts_.reserve(org.bodyParts_.size());
    for(auto part : org.bodyParts_){
        if(!part){
            bodyParts_.push_back(nullptr);
        } else {
            auto newPart = createBodyPartFunc();
            copyBodyPartFunc(newPart, part, cloneMap);
            bodyParts_.push_back(newPart);
        }
    }
        
    mainBodyPartIndex_ = org.mainBodyPartIndex_;
}


Referenced* KinematicBodySet::doClone(CloneMap* cloneMap) const
{
    return new KinematicBodySet(*this, cloneMap);
}


void KinematicBodySet::copyBodyPart(BodyPart* newBodyPart, BodyPart* orgBodyPart, CloneMap* cloneMap)
{
    if(orgBodyPart->isLinkKinematicsKit()){
        initializeBodyPart(newBodyPart, orgBodyPart->linkKinematicsKit()->clone(cloneMap));
    } else if(orgBodyPart->isJointTraverse()){
        initializeBodyPart(newBodyPart, orgBodyPart->jointTraverse());
    }
}


void KinematicBodySet::initializeBodyPart(BodyPart* bodyPart, LinkKinematicsKit* kinematicsKit)
{
    if(kinematicsKit != bodyPart->linkKinematicsKit_){
        bodyPart->linkKinematicsKit_ = kinematicsKit;
        bodyPart->connections.disconnect();
        bodyPart->connections.add(
            kinematicsKit->sigFrameSetChange().connect(
                [this](){ sigFrameSetChange_(); }));
        bodyPart->connections.add(
            kinematicsKit->sigPositionError().connect(
                [this](const Isometry3& T_frameCoordinate){ sigPositionError_(T_frameCoordinate); }));
    }

    bodyPart->jointTraverse_.reset();
}


void KinematicBodySet::initializeBodyPart(BodyPart* bodyPart, std::shared_ptr<JointTraverse> jointTraverse)
{
    bodyPart->jointTraverse_ = jointTraverse;

    if(bodyPart->linkKinematicsKit_){
        bodyPart->linkKinematicsKit_.reset();
        bodyPart->connections.disconnect();
    }
}


KinematicBodySet::BodyPart* KinematicBodySet::findOrCreateBodyPart(int index)
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


void KinematicBodySet::setBodyPart(int index, LinkKinematicsKit* kinematicsKit)
{
    if(kinematicsKit){
        auto bodyPart = findOrCreateBodyPart(index);
        initializeBodyPart(bodyPart, kinematicsKit);
    } else {
        clearBodyPart(index);
    }
}


void KinematicBodySet::setBodyPart(int index, std::shared_ptr<JointTraverse> jointTraverse)
{
    if(jointTraverse){
        auto bodyPart = findOrCreateBodyPart(index);
        initializeBodyPart(bodyPart, jointTraverse);
    } else {
        clearBodyPart(index);
    }
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

