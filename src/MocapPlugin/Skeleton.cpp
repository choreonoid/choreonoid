#include "Skeleton.h"

using namespace cnoid;


Skeleton::Skeleton()
{
    scale_ = 1.0;
}


Skeleton::Skeleton(const Skeleton& org)
    : name_(org.name_)
{
    scale_ = org.scale_;

    if(org.rootBone_){
        setRootBone(new Bone(*org.rootBone()));
    }
}


Skeleton::~Skeleton()
{

}


void Skeleton::setRootBone(BonePtr root)
{
    if(rootBone_ == root){
        return;
    }
    rootBone_ = root;
    updateBones();
}


void Skeleton::updateBones()
{
    bones_.clear();
    axes_.clear();
    boneMap.clear();
    updateBonesIter(rootBone_);
}


void Skeleton::updateBonesIter(Bone* bone)
{
    bone->setIndex(bones_.size());
    bones_.push_back(bone);

    bone->setSkeletonAxisIndex(axes_.size());

    for(size_t i=0; i < bone->numAxes(); ++i){
        axes_.push_back(Axis(bone, bone->axis(i)));
    }
    bone->clearAxisDisplacements();
    
    boneMap[bone->name()] = bone;

    for(Bone* child = bone->child(); child; child = child->sibling()){
        updateBonesIter(child);
    }
}
    

Bone* Skeleton::bone(const std::string& name) const
{
    BoneMap::const_iterator p = boneMap.find(name);
    if(p != boneMap.end()){
        return p->second.get();
    }
    return nullptr;
}


void Skeleton::setScale(double s)
{
    scale_ = s;
}

    
void Skeleton::calcForwardKinematics(const Isometry3& T_offset)
{
    calcForwardKinematicsIter(rootBone_, T_offset);
}


void Skeleton::calcForwardKinematicsIter(Bone* bone, const Isometry3& T_parent)
{
    if(scale_ == 1.0){
        bone->T() = T_parent * bone->Tb() * bone->calcJointTransform();
    } else {
        Isometry3 Ts = bone->Tb();
        Ts.translation() *= scale_;
        bone->T() = T_parent * Ts * bone->calcJointTransform();
    }
    for(Bone* child = bone->child(); child; child = child->sibling()){
        calcForwardKinematicsIter(child, bone->T());
    }
}
