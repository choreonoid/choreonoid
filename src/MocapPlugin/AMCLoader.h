#ifndef CNOID_MOCAP_PLUGIN_AMC_LOADER_H
#define CNOID_MOCAP_PLUGIN_AMC_LOADER_H

#include "Skeleton.h"
#include "Bone.h"
#include "SkeletonMotion.h"
#include "CoordAxisMap.h"
#include <cnoid/EasyScanner>
#include <cnoid/NullOut>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT AMCLoader
{
public:
    AMCLoader();

    void enableCoordinateFlipping(bool on);

    SkeletonPtr loadASF(const std::string& filename, std::ostream& os = nullout());
    SkeletonMotionPtr loadAMC(const std::string& filename, std::ostream& os = nullout());

private:
    SkeletonPtr skeleton;
    BonePtr rootBone;
    EasyScanner scanner;
    std::string version;
    std::string name_;
    std::string documentation;
    double massUnit;
    double lengthUnit;
    double toRadian;
    double toMeter;
    CoordAxisMap axisMap;

    class BoneInfo : public Referenced
    {
    public:
        BonePtr bone;
        Vector3 direction;
        double length;
        int channel;

        BoneInfo(Bone* bone) : bone(bone) {
            direction.setZero();
            length = 0.0;
            channel = 0;
        }
    };
    typedef ref_ptr<BoneInfo> BoneInfoPtr;

    std::map<std::string, BoneInfoPtr> boneInfoMap;

    SkeletonMotionPtr skeletonMotion;

    bool readASF(std::ostream& os);
    void readUnits();
    void readRoot();
    void readRootDofOrder();
    void readRotationAxes(std::vector<Bone::Axis>& out_axes);
    void readBoneDofOrder(Bone* bone);
    void readBones();
    void readBone();
    double readLimitValue();
    void readHierarchy();
    void readChildBones(BoneInfo* parentBoneInfo);
    void addEndNodeBones(Bone* bone);
    void updateChannels();

    void checkAmcHeader();
    void readAmcFrames();
    void readBoneJointPosition(const std::string& boneName, int frame);
};

}

#endif
