#include <cnoid/CustomJointPathHandler>
#include <cnoid/CustomJointPathBase>
#include <cnoid/Body>

using namespace std;
using namespace cnoid;

namespace {

enum ModelType { UNSUPPORTED_MODEL, HRP4C };

struct Geometry
{
    double lateralHipJointOffset;
    double lateralKneeOffset;
    double thighLength;
    double thinLength;
};

class HRP4CHandler : public CustomJointPathHandler
{
public:
    int modelType;
    Geometry geom;

    HRP4CHandler();
    virtual BodyHandler* clone() override;
    virtual bool initialize(Body* body, std::ostream& os) override;
    virtual std::shared_ptr<JointPath> getCustomJointPath(Link* baseLink, Link* endLink) override;
};

class HRP4CJointPath : public CustomJointPathBase
{
public:
    const Geometry geom;

    HRP4CJointPath(HRP4CHandler* handler, Link* baseLink, Link* endLink);
    bool calcLegInverseKinematics(const Isometry3& T_global, const Isometry3& T, double sign);
};

}


BodyHandler* HRP4CHandler::clone()
{
    return new HRP4CHandler(*this);
}


HRP4CHandler::HRP4CHandler()
{
    modelType = UNSUPPORTED_MODEL;
}


bool HRP4CHandler::initialize(Body* body, std::ostream& /* os */)
{
    modelType = UNSUPPORTED_MODEL;
    
    Link* L_HIP_Y = nullptr;
    Link* L_KNEE_P = nullptr;
    Link* L_ANKLE_P = nullptr;
    
    if(body->modelName() == "HRP-4C"){
        L_HIP_Y = body->link("L_HIP_Y");
        L_KNEE_P = body->link("L_KNEE_P");
        L_ANKLE_P = body->link("L_ANKLE_P");
        if(L_HIP_Y && L_KNEE_P && L_ANKLE_P){
            modelType = HRP4C;
        }
    }

    if(modelType == HRP4C){
        geom.lateralHipJointOffset = L_HIP_Y->offsetTranslation().y();
        geom.lateralKneeOffset = -L_KNEE_P->offsetTranslation().y();
        geom.thighLength = -L_KNEE_P->offsetTranslation().z();
        geom.thinLength = -L_ANKLE_P->offsetTranslation().z();
    }
    
    return modelType != UNSUPPORTED_MODEL;
}


std::shared_ptr<JointPath> HRP4CHandler::getCustomJointPath(Link* baseLink, Link* endLink)
{
    return make_shared<HRP4CJointPath>(this, baseLink, endLink);
}


HRP4CJointPath::HRP4CJointPath(HRP4CHandler* handler, Link* baseLink, Link* endLink)
    : geom(handler->geom),
      CustomJointPathBase(baseLink, endLink)
{
    bool isReversed;
    if(numJoints() == 6){
        if(checkLinkPath("WAIST", "L_ANKLE_R", isReversed)){
            setCustomInverseKinematics(
                [&](const Isometry3& T_global, const Isometry3& T_relative){
                    return calcLegInverseKinematics(T_global, T_relative, -1.0); }, isReversed);
        } else if(checkLinkPath("WAIST", "R_ANKLE_R", isReversed)){
            setCustomInverseKinematics(
                [&](const Isometry3& T_global, const Isometry3& T_relative){
                    return calcLegInverseKinematics(T_global, T_relative, 1.0); }, isReversed);
        }
    }
}
    

bool HRP4CJointPath::calcLegInverseKinematics(const Isometry3& T_global, const Isometry3& T, double sign)
{
    Vector6 q;
    
    const double L1 = geom.lateralHipJointOffset;
    const double L2 = geom.lateralKneeOffset;
    const double L3 = geom.thighLength;
    const double L4 = geom.thinLength;

    auto R_ah = T.linear().transpose();
    Vector3 p = T.translation();
    p.y() += sign * L1;
    Vector3 p_ah = -R_ah * p;
    
    double c_k = (p_ah.dot(p_ah) - (L2 * L2 + L3 * L3 + L4 * L4)) / (2.0 * L3 * L4);
    if(c_k > 1.0){
        return false;
    }
    q[3] = acos(c_k);
    
    double s_k = sqrt(1.0 - c_k * c_k);
    Vector3 p2(-L3 * s_k, -1.0 * sign * L2, L3 * c_k + L4);

    double a = p_ah.y() * p_ah.y() + p_ah.z() * p_ah.z() - p2.y() * p2.y();
    if(a < 0.0){
        return false;
    }
    q[4] = atan2(p2.x(), p2.z()) - atan2(p_ah.x(), sqrt(a));

    double c_ap = cos(q[4]);
    double s_ap = sin(q[4]);
    q[5] = -atan2(p_ah.z(), p_ah.y()) + atan2(s_ap * p2.x() + c_ap * p2.z(), p2.y());

    double c_kap = cos(q[3] + q[4]);
    double s_kap = sin(q[3] + q[4]);
    double c_ar = cos(q[5]);
    double s_ar = sin(q[5]);

    Matrix3 R_ak;
    R_ak <<
        c_kap,         0.0,  -s_kap,
        s_ar * s_kap,  c_ar,  s_ar * c_kap,
        c_ar * s_kap, -s_ar,  c_ar * c_kap;

    Matrix3 R_kh = R_ak.transpose() * R_ah;

    q[0] = atan2(-R_kh(1,0), R_kh(1,1));
    q[1] = atan2(R_kh(1,2), sqrt(R_kh(1,0) * R_kh(1,0) + R_kh(1,1) * R_kh(1,1)));
    q[2] = atan2(-R_kh(0,2), R_kh(2,2));

    copyJointDisplacements(q.data());
    calcForwardKinematics();

    return true;
}


CNOID_IMPLEMENT_BODY_HANDLER_FACTORY(HRP4CHandler)
