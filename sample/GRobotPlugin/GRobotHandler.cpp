#include <cnoid/CustomJointPathHandler>
#include <cnoid/CustomJointPathBase>
#include <cnoid/Body>
#include <cnoid/EigenUtil>

using namespace std;
using namespace cnoid;

namespace {

enum ModelType { UNSUPPORTED_MODEL = 0, GR001 = 1, RIC30 = 2 };

struct Geometry {
    double l00;
    double l0;
    double l1;
    double lo1;
    double l2;
    double lo2;
    double l3;
    double l4;
    double lo5;
    // joint angle offset of HIP_P and KNEE_P affected by lo3.
    double q2q3offset;
};

class GRobotHandler : public CustomJointPathHandler
{
public:
    int modelType;
    Geometry geom;

    virtual BodyHandler* clone() override { return new GRobotHandler(*this); }
    virtual bool initialize(Body* body, std::ostream& os) override;
    virtual std::shared_ptr<JointPath> getCustomJointPath(Link* baseLink, Link* endLink) override;
};

class GRobotJointPath : public CustomJointPathBase
{
public:
    const Geometry& geom;
    Vector3 p;
    Vector3 rpy;
    Vector6 q;
    double sign;
    double cy;
    double sy;
    double cp;
    double sp;
    double cr;
    double sr;
    double slo1;
    Matrix4d Af;
    Matrix4d TB0;
    Matrix4d TB1;
    Matrix4d TB2;
    Matrix4d TB3;
    Matrix4d TB4;
    Matrix4d TB5;
    Matrix4d TB6;
    Matrix4d TBF;
    Vector6 dp;
    Vector6 dq;

    GRobotJointPath(GRobotHandler* handler, Link* baseLink, Link* endLink);
    bool calcLegInverseKinematics(const Position& T, double sign);
    bool calcEndPositionDifference(double sign);
};

}

bool GRobotHandler::initialize(Body* body, std::ostream& os)
{
    if(body->modelName() == "GR001"){
        modelType = GR001;
    } else if(body->modelName() == "RIC30"){
        modelType = RIC30;
    } else {
        return false;
    }

    switch(modelType){

    case GR001:
        geom.l00 = 0.0;
        geom.l0  = 0.0221;
        geom.l1  = 0.028;
        geom.lo1 = 0.0025;
        geom.l2  = 0.029;
        geom.lo2 = 0.005;
        geom.l3  = 0.048;
        geom.l4  = 0.059;
        geom.lo5 = 0.0005;
        geom.q2q3offset = 0.0;
        break;

    case RIC30:
        geom.l00 = 0.002;
        geom.l0  = 0.0245;
        geom.l1  = 0.02;
        geom.lo1 = 0.0;
        geom.l2  = 0.0026;
        geom.lo2 = 0.0;

        // x-offset from HIP_P to KNEE_P.
        // l3 and q2q3offset are arranged by this value.
        const double lo3 = -0.001;
            
        const double l3  = 0.0635;
        geom.l3 = sqrt(l3*l3 + lo3*lo3);
        geom.q2q3offset = atan(lo3 / l3);
                
        geom.l4  = 0.067;
        geom.lo5 = 0.0;
        break;
    }

    return true;
}


std::shared_ptr<JointPath> GRobotHandler::getCustomJointPath(Link* baseLink, Link* endLink)
{
    return make_shared<GRobotJointPath>(this, baseLink, endLink);
}


GRobotJointPath::GRobotJointPath(GRobotHandler* handler, Link* baseLink, Link* endLink)
    : geom(handler->geom),
      CustomJointPathBase(baseLink, endLink)
{
    bool isReversed;
    if(numJoints() == 6){
        if(checkLinkPath("WAIST", "L_ANKLE_R", isReversed)){
            setCustomInverseKinematics(
                [&](const Position& T_global, const Position& T_relative){
                    return calcLegInverseKinematics(T_relative,  1.0); }, isReversed);
        } else if(checkLinkPath("WAIST", "R_ANKLE_R", isReversed)){
            setCustomInverseKinematics(
                [&](const Position& T_global, const Position& T_relative){
                    return calcLegInverseKinematics(T_relative, -1.0); }, isReversed);
        }
    }
}
    

bool GRobotJointPath::calcLegInverseKinematics(const Position& T, double sign)
{
    rpy = rpyFromRot(T.linear());
    p = T.translation();
    p.z() += geom.l00;

    cy = cos(rpy[2]);
    sy = sin(rpy[2]);
    cp = cos(rpy[1]);
    sp = sin(rpy[1]);
    cr = cos(rpy[0]);
    sr = sin(rpy[0]);

    Vector3 O1(0.0, sign * geom.l0, -geom.l1);
    const Vector3& F = p;
    Vector3 V1 = F - O1;
    Vector3 XF(cy * cp, sy * cp, -sp);
    Vector3 V1xXF = V1.cross(XF);
    Vector3 Z2 = -sign * V1xXF / V1xXF.norm();

    q[0] = atan2(-sign * Z2[0], sign * Z2[1]);
    q[1] = asin(-sign * Z2[2]);

    double c1 = cos(q[0]);
    double s1 = sin(q[0]);
    double c2 = cos(q[1]);
    double s2 = sin(q[1]);
    double s1s2 = s1 * s2;
    double c1s2 = c1 * s2;
    slo1 = sign * geom.lo1;

    TB2 <<
         s1s2, -sign * c1, -sign * s1 * c2,  slo1 * s1 + geom.l2 * c1 + geom.lo2 * s1s2,
        -c1s2, -sign * s1,  sign * c1 * c2,  sign * geom.l0 - slo1 * c1 + geom.l2 * s1 - geom.lo2 * c1s2,
        -c2,    0.0,       -sign * s2,      -geom.l1 - geom.lo2 * c2,
         0.0,   0.0,        0.0,             1.0;

    Vector3 V2 = (TB2.inverse() * Vector4d(F[0], F[1], F[2], 1.0)).head(3);
    double D = (V2.squaredNorm() - geom.l3 * geom.l3 - geom.l4 * geom.l4) / (2.0 * geom.l3 * geom.l4);

    if(fabs(D) > 1.0){
        return false;
    }

    q[3] = atan2(-sign * sqrt(1.0 - D * D), D);
    double c4 = cos(q[3]);
    double s4 = sin(q[3]);
        
    double beta = atan2(-V2[1], sqrt(V2[0] * V2[0] + V2[2] * V2[2]));
    double alpha = atan2(geom.l4 * s4, geom.l3 + geom.l4 * c4);
    q[2] = -(beta - alpha);
        
    q[3] = -q[3];
        
    double c3 = cos(q[2]);
    double s3 = sin(q[2]);
    double q2q3 = q[2] + q[3];
    double c34 = cos(q2q3);
    double s34 = sin(q2q3);
        
    Matrix4d T24;
    T24 <<
        c34,  s34,  0, geom.l3 * c3 + geom.l4 * c34,
        s34, -c34,  0, geom.l3 * s3 + geom.l4 * s34,
        0.0,  0.0, -1, 0.0,
        0.0,  0.0,  0, 1.0;
        
    TB4.noalias() = TB2 * T24;
    
    double spsr = sp * sr;
    double spcr = sp * cr;
    
    TBF <<
         cy * cp, -sy * cr + cy * spsr,  sy * sr + cy * spcr, p.x(),
         sy * cp,  cy * cr + sy * spsr, -cy * sr + sy * spcr, p.y(),
        -sp,       cp * sr,              cp * cr,             p.z(),
         0,        0,                    0,                   1.0;
        
    Matrix4d T4F;
    T4F.noalias() = TB4.inverse() * TBF;
    
    q[4] = atan2(-sign * T4F(0,0),  sign * T4F(1,0));
    q[5] = atan2( sign * T4F(2,2), -sign * T4F(2,1));

    // Numerical refining
        
    TB0 <<
        0.0, -1.0, 0.0, 0.0,
        1.0,  0.0, 0.0, sign * geom.l0,
        0.0,  0.0, 1.0, 0.0,
        0.0,  0.0, 0.0, 1.0;
        
    Af <<
        0.0, 1.0, 0.0, 0.0,
        -1.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0;

    bool solved = false;

    int i;
    for(i=0; i < 30; ++i){

        if(calcEndPositionDifference(sign)){
            solved = true;
            break;
        }

        // Jacobian Calculation

        Vector3 Z0 = TB0.block<3,1>(0,2);
        Vector3 Z1 = TB1.block<3,1>(0,2);
        Vector3 Z2 = TB2.block<3,1>(0,2);
        Vector3 Z3 = TB3.block<3,1>(0,2);
        Vector3 Z4 = TB4.block<3,1>(0,2);
        Vector3 Z5 = TB5.block<3,1>(0,2);
        
        Vector3 O0 = TB0.block<3,1>(0,3);
        Vector3 O1 = TB1.block<3,1>(0,3);
        Vector3 O2 = TB2.block<3,1>(0,3);
        Vector3 O3 = TB3.block<3,1>(0,3);
        Vector3 O4 = TB4.block<3,1>(0,3);
        Vector3 O5 = TB5.block<3,1>(0,3);
        Vector3 O6 = TB6.block<3,1>(0,3);

        typedef Eigen::Matrix<double, 6, 6> Matrix6;
        
        Matrix6 J;
        J <<
            Z0.cross(O6-O0), Z1.cross(O6-O1), Z2.cross(O6-O2), Z3.cross(O6-O3), Z4.cross(O6-O4), Z5.cross(O6-O5),
            Z0,              Z1,              Z2,              Z3,              Z4,              Z5             ;
            
        // Levenberg-Marquardt Method
            
        const double lambda = 0.001;
            
        Matrix6 C;
        C.noalias() = J.transpose() * (J * J.transpose() + Matrix6::Identity() * lambda * lambda).inverse();
            
        dq.noalias() = C * dp;
        q += dq;

        if(dq.norm() <= 1.0e-5){
            break;
        }
    }

    if(!solved){
        solved = calcEndPositionDifference(sign);
    }

    if(solved){
        q[2] += sign * geom.q2q3offset;
        q[3] -= sign * geom.q2q3offset;

        copyJointDisplacements(q.data());
        calcForwardKinematics();
    }

    return solved;
}


bool GRobotJointPath::calcEndPositionDifference(double sign)
{
    // Direct Kinematics Error Calculation
    double c1 = cos(q[0]);
    double s1 = sin(q[0]);
    double c2 = cos(q[1]);
    double s2 = sin(q[1]);
    double c3 = cos(q[2]);
    double s3 = sin(q[2]);
    double c4 = cos(q[3]);
    double s4 = sin(q[3]);
    double c5 = cos(q[4]);
    double s5 = sin(q[4]);
    double c6 = cos(q[5]);
    double s6 = sin(q[5]);
    
    Matrix4d A1, A2, A3, A4, A5, A6;

    A1 <<
        c1,   0.0, -s1,  -slo1 * c1,
        s1,   0.0,  c1,  -slo1 * s1,
        0.0, -1.0,  0.0, -geom.l1,
        0.0,  0.0,  0.0,  1.0;
    
    A2 <<
        -s2,  0.0, sign * c2, -geom.lo2 * s2,
         c2,  0.0, sign * s2,  geom.lo2 * c2,
        0.0, sign, 0.0,       -geom.l2,
        0.0,  0.0, 0.0,        1.0;
    
    A3 <<
        c3, -s3,  0.0, geom.l3 * c3,
        s3,  c3,  0.0, geom.l3 * s3,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0;
        
    A4 <<
        c4,  s4,   0.0, geom.l4 * c4,
        s4, -c4,   0.0, geom.l4 * s4,
        0.0, 0.0, -1.0, 0.0,
        0.0, 0.0,  0.0, 1.0;
        
    A5 <<
        c5,   0.0, -sign * s5, -geom.lo5 * c5,
        s5,   0.0,  sign * c5, -geom.lo5 * s5,
        0.0, -sign,  0.0,       0.0,
        0.0,  0.0,   0.0,       1.0;
        
    A6 <<
        -s6,  0.0, -c6,  0.0,
         c6,  0.0, -s6,  0.0,
        0.0, -1.0,  0.0, 0.0,
        0.0,  0.0,  0.0, 1.0;
        
    TB1.noalias() = TB0 * A1;
    TB2.noalias() = TB1 * A2;
    TB3.noalias() = TB2 * A3;
    TB4.noalias() = TB3 * A4;
    TB5.noalias() = TB4 * A5;
    TB6.noalias() = TB5 * A6;
    TBF.noalias() = TB6 * Af; 
    
    double yaw = atan2(TBF(1,0), TBF(0,0));
    double pitch = asin(-TBF(2,0));
    double roll = atan2(TBF(2,1), TBF(2,2));
    double dYaw = rpy[2] - yaw;
    double dPitch = rpy[1] - pitch;
    double dRoll = rpy[0] - roll;
    
    Vector3 pi = TBF.block<3,1>(0,3);

    dp <<
        (p - pi),
        (-sy * dPitch + cy * cp * dRoll),
        (cy * dPitch + sy * cp * dRoll),
        (dYaw - sp * dRoll);

    double errsqr = dp.head(3).squaredNorm() + dp.tail(3).squaredNorm();
    return (errsqr < 1.0e-6 * 1.0e-6);
}


CNOID_IMPLEMENT_BODY_HANDLER_FACTORY(GRobotHandler)
