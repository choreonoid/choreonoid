#include "Bone.h"

using namespace cnoid;


Bone::Bone()
{
    T_.setIdentity();
    Tb_.setIdentity();
    Ro_.setIdentity();
    parent_ = nullptr;
    boneIndex_ = -1;
    skeletonAxisIndex_ = -1;
    isEulerAngleMode_ = false;
}


Bone::Bone(const Bone& org)
    : axes_(org.axes_),
      axisDisplacements_(org.axisDisplacements_),
      name_(org.name_)
{
    T_ = org.T_;
    Tb_ = org.Tb_;
    Ro_ = org.Ro_;
    parent_ = nullptr;
    boneIndex_ = -1;
    skeletonAxisIndex_ = -1;
    isEulerAngleMode_ = org.isEulerAngleMode_;
    
    for(Bone* orgChild = org.child(); orgChild; orgChild = orgChild->sibling()){
        appendChild(new Bone(*orgChild));
    }
}


Bone::~Bone()
{

}


void Bone::appendChild(Bone* newChild)
{
    Bone* lastChild = child();

    for(Bone* bone = child(); bone; bone = bone->sibling()){
        lastChild = bone;
    }

    if(lastChild){
        lastChild->sibling_ = newChild;
    } else {
        child_ = newChild;
    }

    newChild->parent_ = this;
}


void Bone::setAxes(const std::vector<Axis>& axes)
{
    axes_ = axes;
    axisDisplacements_.resize(axes.size());
}


void Bone::clearAxisDisplacements()
{
    axisDisplacements_.clear();
    axisDisplacements_.resize(axes_.size(), 0.0);
}


Isometry3 Bone::calcAxisSetTransform
(const std::vector<Axis>& axes, const double* displacements, bool isEulerAngleMode)
{
    Isometry3 T_local = Isometry3::Identity();
    
    for(size_t i=0; i < axes.size(); ++i){
        Axis axis = axes[i];
        double q = displacements[i];
        if(checkAxisType(axis) == TranslationAxis){
            T_local.translation()[axis] = q;
        } else { // rotation
            if(isEulerAngleMode){
                T_local.linear() = AngleAxis(q, Vector3::Unit(axis - RX)) * T_local.linear();
            } else {
                T_local.linear() = T_local.linear() * AngleAxis(q, Vector3::Unit(axis - RX));
            }
        }
    }

    return T_local;
}


Isometry3 Bone::calcJointTransform() const
{
    Isometry3 T = calcAxisSetTransform(axes_, &axisDisplacements_.front(), isEulerAngleMode_);
    T.linear() = Ro_ * T.linear() * Ro_.transpose();
    return T;
}
