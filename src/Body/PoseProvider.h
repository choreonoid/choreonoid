/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_POSE_PROVIDER_H
#define CNOID_BODY_POSE_PROVIDER_H

#include "Body.h"
#include <cnoid/EigenTypes>
#include <cnoid/stdx/optional>
#include <vector>

namespace cnoid {

class PoseProvider
{
public:
    virtual ~PoseProvider() { };
    virtual Body* body() const = 0;
    virtual double beginningTime() const = 0;
    virtual double endingTime() const = 0;
    virtual bool seek(double time) = 0;
    virtual bool seek(double time, int waistLinkIndex, const Vector3& waistTranslation) = 0;
    virtual int baseLinkIndex() const = 0;
    virtual bool getBaseLinkPosition(Position& out_T) const = 0;
    virtual void getJointPositions(std::vector<stdx::optional<double>>& out_q) const = 0;
    virtual stdx::optional<Vector3> ZMP() const = 0;

#ifdef CNOID_BACKWARD_COMPATIBILITY
    bool getBaseLinkPosition(Vector3& out_p, Matrix3& out_R) const {
        Position T;
        if(getBaseLinkPosition(T)){
            out_p = T.translation();
            out_R = T.linear();
            return true;
        }
        return false;
    }
#endif
};
}

#endif
