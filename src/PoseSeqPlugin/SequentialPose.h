#ifndef CNOID_POSE_SEQ_PLUGIN_SEQUENTIAL_POSE_H
#define CNOID_POSE_SEQ_PLUGIN_SEQUENTIAL_POSE_H

#include "AbstractPose.h"
#include "exportdecl.h"

namespace cnoid {

class PoseSeq;

class CNOID_EXPORT SequentialPose
{
public:
    SequentialPose(const SequentialPose& org, CloneMap* cloneMap = nullptr);

    AbstractPose* pose() { return pose_; }
    const AbstractPose* pose() const { return pose_; }

    template <class PoseType>
    const PoseType* get() const {
        return dynamic_cast<PoseType*>(pose_.get());
    }

    template <class PoseType>
    PoseType* get() {
        return dynamic_cast<PoseType*>(pose_.get());
    }

    double time() const {
        return time_;
    }

    /**
       This function sets the default transition time.
       When the time length between the previous pose and this pose
       is longer than the default transition time, the sysytem uses this
       time to interpolate the two poses. If the time length between the
       two poses is shorter than the default transition time, the former length
       is used for the interpolation.
       If the default transition time is zero, the interpolation is always done
       using the time length between the two poses.
    */
    void setMaxTransitionTime(double time){
        maxTransitionTime_ = time;
    }

    double maxTransitionTime() const {
        return maxTransitionTime_;
    }

protected:
    SequentialPose(AbstractPose* pose, double time);

private:
    AbstractPosePtr pose_;
    double time_;
    double maxTransitionTime_;

    friend class PoseSeq;
};

}

#endif
