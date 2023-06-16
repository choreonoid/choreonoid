#ifndef CNOID_MOCAP_PLUGIN_SKELETON_H
#define CNOID_MOCAP_PLUGIN_SKELETON_H

#include "Bone.h"
#include <cnoid/Referenced>
#include <string>
#include <vector>
#include <map>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT Skeleton : public Referenced
{
public:
    Skeleton();
    Skeleton(const Skeleton& org);
    ~Skeleton();

    const std::string& name() { return name_; }
    void setName(const std::string& name) { name_ = name; }

    void setRootBone(BonePtr root);
    Bone* rootBone() const { return rootBone_; }

    void updateBones();

    int numBones() const { return bones_.size(); }
    Bone* bone(int index) const { return bones_[index]; }
    Bone* bone(const std::string& name) const;
    const std::vector<BonePtr>& bones() const { return bones_; }

    void setScale(double s);
    double scale() const { return scale_; }

    struct Axis
    {
        BonePtr bone;
        Bone::Axis axis;
        Axis(Bone* bone, Bone::Axis axis)
            : bone(bone), axis(axis) { }
    };

    const std::vector<Axis>& axes() { return axes_; }
    int numAxes() const { return axes_.size(); }
    const Axis& axis(int index) const { return axes_[index]; }

    void calcForwardKinematics(const Isometry3& T_offset);

private:
    BonePtr rootBone_;
    std::vector<BonePtr> bones_;
    std::vector<Axis> axes_;
    double scale_;
    std::string name_;
    typedef std::map<std::string, BonePtr> BoneMap;
    BoneMap boneMap;
        
    void updateBonesIter(Bone* bone);
    void calcForwardKinematicsIter(Bone* bone, const Isometry3& T_parent);
};

typedef ref_ptr<Skeleton> SkeletonPtr;

}

#endif
