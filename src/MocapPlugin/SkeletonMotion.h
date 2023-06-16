#ifndef CNOID_MOCAP_PLUGIN_SKELETON_MOTION_H
#define CNOID_MOCAP_PLUGIN_SKELETON_MOTION_H

#include "Skeleton.h"
#include "CoordAxisMap.h"
#include <cnoid/MultiValueSeq>
#include <cnoid/EigenTypes>
#include <vector>
#include "exportdecl.h"

namespace cnoid {

class EasyScanner;

/**
   \todo Channel ordres should be exchanged when the coordinate is fliped,
   or quaternion should be used as a common format
*/
class CNOID_EXPORT SkeletonMotion : public MultiValueSeq
{
public:
    typedef std::shared_ptr<SkeletonMotion> Ptr;

    SkeletonMotion();
    SkeletonMotion(SkeletonPtr skeleton);
    SkeletonMotion(const SkeletonMotion& org);
    ~SkeletonMotion();

    Skeleton* skeleton() const { return skeleton_.get(); }
    void resetSkeleton(SkeletonPtr skeleton);

    void setPositionOffset(const Isometry3& T_offset);
    const Isometry3& positionOffset() const { return T_offset_; }

    void setTranslation(const Vector3& translation);
    Vector3 translation() const { return T_offset_.translation(); }

    void setYawRotation(double angle);
    double yawRotation() const;

    bool setFrameToSkeleton(int frame);
    int currentFrame() const { return currentFrame_; }

    void setAxisMap(const CoordAxisMap& axisMap){ this->axisMap = axisMap; }

    void convertToStandardForm();

    bool loadBVH(const std::string& filename, double scale = 1.0);
    bool saveBVH(const std::string& filename);

    const std::string& messages() { return messages_; }

private:
    SkeletonPtr skeleton_;
    CoordAxisMap axisMap;
    int currentFrame_;
    Isometry3 T_offset_;
    std::string messages_;

    Bone* readBvhJoint(EasyScanner& scanner, bool isEnd);
    int readBvhChannels(EasyScanner& scanner, Bone* bone);
    void readBvhMotion(EasyScanner& scanner);
    void writeBvhBone(Bone* bone, std::ostream& os, int depth, std::string& indent);
};

typedef SkeletonMotion::Ptr SkeletonMotionPtr;

}

#endif
