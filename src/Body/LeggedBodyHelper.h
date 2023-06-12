#ifndef CNOID_BODY_LEGGED_BODY_HELPER_H
#define CNOID_BODY_LEGGED_BODY_HELPER_H

#include "Body.h"
#include "InverseKinematics.h"
#include <memory>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT LeggedBodyHelper : public Referenced
{
public:
    LeggedBodyHelper();
    LeggedBodyHelper(Body* body);
    LeggedBodyHelper(const LeggedBodyHelper& org);

    bool isValid() const { return isValid_; }
    virtual bool resetBody(Body* body);
        
    virtual ~LeggedBodyHelper();

    Body* body() const { return body_; }

    int numFeet() const { return footInfos.size(); }

    //! The foot link indices should be the same as the following IDs.
    enum FootID {
        Left = 0,
        Right = 1
    };

    Link* footLink(int index) const { return footInfos[index].link; }

    Link* kneePitchJoint(int footIndex) const { return footInfos[footIndex].kneePitchJoint; }

    std::shared_ptr<InverseKinematics> getFootBasedIK(Link* targetLink);
    
    bool doLegIkToMoveCm(const Vector3& c, bool onlyProjectionToFloor = false);
    bool setStance(double width, Link* baseLink);

    const Vector3& centerOfSoleLocal(int footIndex) const { return footInfos[footIndex].soleCenter; }
    Vector3 centerOfSole(int footIndex) const;
    Vector3 centerOfSoles() const;

    const Vector3& homeCopOfSoleLocal(int footIndex) const { return footInfos[footIndex].homeCop; }
    Vector3 homeCopOfSole(int footIndex) const;
    Vector3 homeCopOfSoles() const;

    const Isometry3& toeOffset(int footIndex) const { return footInfos[footIndex].toeOffset; };
    
private:
    BodyPtr body_;
    bool isValid_;

    struct FootInfo
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
        LinkPtr link;
        LinkPtr kneePitchJoint;
        Vector3 homeCop;
        Vector3 soleCenter;
        Isometry3 toeOffset;
    };

    std::vector<FootInfo, Eigen::aligned_allocator<FootInfo>> footInfos;
};

typedef ref_ptr<LeggedBodyHelper> LeggedBodyHelperPtr;

CNOID_EXPORT LeggedBodyHelper* getLeggedBodyHelper(Body* body);

}

#endif
