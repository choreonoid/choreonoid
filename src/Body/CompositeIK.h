/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_COMPOSITE_IK_H
#define CNOID_BODY_COMPOSITE_IK_H

#include "Body.h"
#include "InverseKinematics.h"
#include "exportdecl.h"

namespace cnoid {

class JointPath;
typedef boost::shared_ptr<JointPath> JointPathPtr;

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
    int numJointPaths() const { return pathList.size(); }
    JointPathPtr jointPath(int index) const { return pathList[index].path; }
    Link* baseLink(int index) const { return pathList[index].endLink; }

    bool hasAnalyticalIK() const;
    virtual bool calcInverseKinematics(const Vector3& p, const Matrix3& R);
        
private:
    BodyPtr body_;
    Link* targetLink_;
    struct PathInfo {
        JointPathPtr path;
        Link* endLink;
        Vector3 p_given;
        Matrix3 R_given;
    };
    std::vector<PathInfo> pathList;
    std::vector<double> q0;
    bool isAnalytical_;
};

typedef boost::shared_ptr<CompositeIK> CompositeIKPtr;

}

#endif
