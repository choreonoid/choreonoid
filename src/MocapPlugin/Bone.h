#ifndef CNOID_MOCAP_PLUGIN_BONE_H
#define CNOID_MOCAP_PLUGIN_BONE_H

#include <cnoid/Referenced>
#include <cnoid/EigenTypes>
#include <vector>
#include "exportdecl.h"

namespace cnoid {

class Skeleton;

class Bone;
typedef ref_ptr<Bone> BonePtr;

class CNOID_EXPORT Bone : public Referenced
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Bone();
    Bone(const Bone& org);
    ~Bone();

    void setName(const std::string& name) { name_ = name; }
    const std::string& name() { return name_; }

    int index() const { return boneIndex_; }
    void setIndex(int index) { boneIndex_ = index; }
        
    void appendChild(Bone* newChild);
    Bone* parent() const { return parent_; };
    Bone* sibling() const { return sibling_.get(); };
    Bone* child() const { return child_.get(); }

    // Global position
    Isometry3& T() { return T_; }
    const Isometry3& T() const { return T_; }
    const Isometry3& position() const { return T_; }
    Isometry3::ConstTranslationPart translation() const { return T_.translation(); }
    Isometry3::ConstLinearPart rotation() const { return T_.linear(); }

    // Local offset from the parent bone origin
    Isometry3& Tb() { return Tb_; }
    const Isometry3& Tb() const { return Tb_; }
    const Isometry3& offsetPosition() const { return Tb_; }
    Isometry3::ConstTranslationPart offsetTranslation() const { return Tb_.translation(); }
    Isometry3::ConstLinearPart offsetRotation() const { return Tb_.linear(); }
    const Matrix3& rotationOrientation() const { return Ro_; }

    void setOffsetPosition(const Isometry3& T){ Tb_ = T; }
    void setOffsetTranslation(const Vector3& p){ Tb_.translation() = p; }
    void setOffsetRotation(const Matrix3& R){ Tb_.linear() = R; }
    void setRotationOrientation(const Matrix3& R){ Ro_ = R; }
    
    enum Axis { TX, TY, TZ, RX, RY, RZ };
    enum AxisType { TranslationAxis, RotationAxis };

    int numAxes() const { return axes_.size(); }
    Axis axis(int localIndex) const { return axes_[localIndex]; }
    const std::vector<Axis>& axes() const { return axes_; }
    void appendAxis(Axis axis){ axes_.push_back(axis); }
    void setAxes(const std::vector<Axis>& axes);
    void setEulerAngleMode(bool on) { isEulerAngleMode_ = on; }
    bool isEulerAngleMode() const { return isEulerAngleMode_; }
    
    static AxisType checkAxisType(Axis axis){
        return (axis <= TZ) ? TranslationAxis : RotationAxis;
    }

    std::vector<double>& axisDisplacements() { return axisDisplacements_; }
    void clearAxisDisplacements();
    double& q(int axisIndex) { return axisDisplacements_[axisIndex]; }
    const double& q(int axisIndex) const { return axisDisplacements_[axisIndex]; }

    //! Global index of the bone's top axis in the whole skeleton
    int skeletonAxisIndex() const { return skeletonAxisIndex_; }
    void setSkeletonAxisIndex(int index){ skeletonAxisIndex_ = index; }

    static Isometry3 calcAxisSetTransform(
        const std::vector<Axis>& axes, const double* displacements, bool isEulerAngleMode = false);

    Isometry3 calcJointTransform() const;

private:
    Isometry3 T_;
    Isometry3 Tb_;
    Matrix3 Ro_; // Local orientation of rotation

    Bone* parent_;
    BonePtr sibling_;
    BonePtr child_;
    
    int boneIndex_;
    int skeletonAxisIndex_;
    
    std::vector<Axis> axes_;
    std::vector<double> axisDisplacements_;
    bool isEulerAngleMode_;
    
    std::string name_;
};

}

#endif
