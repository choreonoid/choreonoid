/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_COMPOSITE_IK_H
#define CNOID_BODY_COMPOSITE_IK_H

#include "Body.h"
#include "JointPath.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT CompositeIK : public InverseKinematics
{
public:
    CompositeIK();
    CompositeIK(Body* body, Link* targetLink);
    ~CompositeIK();

    void reset(Body* body, Link* targetLink);
    bool addBaseLink(Link* link);
    void setMaxIKerror(double e);

    Body* body() { return body_; }
    Link* targetLink() { return targetLink_; }
    int numJointPaths() const { return paths.size(); }
    std::shared_ptr<JointPath> jointPath(int index) const { return paths[index]; }
    Link* baseLink(int index) const { return paths[index]->endLink(); }

    bool hasAnalyticalIK() const { return hasAnalyticalIK_; }

    virtual bool calcInverseKinematics(const Position& T) override;

    //! deprecated
    bool calcInverseKinematics(const Vector3& p, const Matrix3& R) {
        return InverseKinematics::calcInverseKinematics(p, R);
    }
        
private:
    BodyPtr body_;
    Link* targetLink_;
    std::vector<std::shared_ptr<JointPath>> paths;
    std::vector<double> q0;
    bool hasAnalyticalIK_;
};

}

#endif
