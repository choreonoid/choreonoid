#include "CustomJointPathBase.h"

using namespace cnoid;


CustomJointPathBase::CustomJointPathBase(Link* baseLink, Link* endLink)
    : JointPath(baseLink, endLink)
{
    isReversed_ = false;
}


bool CustomJointPathBase::checkLinkPath(const std::string& from, const std::string& to, bool& out_isReversed) const
{
    const auto& baseLinkName = baseLink()->name();
    const auto& endLinkName = endLink()->name();
    if(baseLinkName == from && endLinkName == to){
        out_isReversed = false;
        return true;
    } else if(baseLinkName == to && endLinkName == from){
        out_isReversed = true;
        return true;
    }
    return false;
}


void CustomJointPathBase::setCustomInverseKinematics(InverseKinematicsFunc func, bool isReversed)
{
    calcCustomInverseKinematics = func;
    isReversed_ = isReversed;
}


bool CustomJointPathBase::calcInverseKinematics(const Position& T)
{
    if(isNumericalIkEnabled() || !calcCustomInverseKinematics){
        return JointPath::calcInverseKinematics(T);
    }

    bool solved = false;
    if(!isReversed_){
        Position T_relative = baseLink()->T().inverse(Eigen::Isometry) * T;
        solved = calcCustomInverseKinematics(T, T_relative);
    } else {
        Position T_relative = T.inverse(Eigen::Isometry) * baseLink()->T();
        solved = calcCustomInverseKinematics(T, T_relative);
    }

    return solved;
}


bool CustomJointPathBase::hasCustomIK() const
{
    return calcCustomInverseKinematics != nullptr;
}
