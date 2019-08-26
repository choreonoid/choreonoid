#include "EigenUtil.h"
#include <fmt/format.h>

#include <iostream>
using namespace std;


namespace {

const bool USE_OLD_RPY_FROM_ROT_IMPLEMENTATION = false;

}

namespace cnoid {

Matrix3 rotFromRpy(double r, double p, double y)
{
    const double cr = cos(r);
    const double sr = sin(r);
    const double cp = cos(p);
    const double sp = sin(p);
    const double cy = cos(y);
    const double sy = sin(y);

    Matrix3 R;
    R << cp*cy, sr*sp*cy - cr*sy, cr*sp*cy + sr*sy,
         cp*sy, sr*sp*sy + cr*cy, cr*sp*sy - sr*cy,
         -sp  , sr*cp           , cr*cp;

    return R;
}


Vector3 rpyFromRot(const Matrix3& R)
{
    const double epsilon = 1.0e-6;
    double roll, pitch, yaw;
    
    if(!USE_OLD_RPY_FROM_ROT_IMPLEMENTATION){
        double a = sqrt(R(2,1) * R(2,1) + R(2,2) * R(2,2));
        pitch = atan2(-R(2,0), a);
        if(pitch > M_PI / 2.0 - epsilon){
            roll = atan2(-R(1,2), R(1,1));
            yaw = 0.0;
        } else if(pitch < -M_PI / 2.0 + epsilon){
            roll = atan2(-R(1,2), R(1,1));
            yaw = 0.0;
        } else {
            roll = atan2(R(2,1), R(2,2));
            yaw = atan2(R(1,0), R(0,0));
        }
    } else {
        if((fabs(R(0,0)) < fabs(R(2,0))) && (fabs(R(1,0)) < fabs(R(2,0)))){
            // cos(p) is nearly = 0
            double sp = -R(2,0);
            if(sp < -1.0){
                sp = -1.0;
            } else if(sp > 1.0){
                sp = 1.0;
            }
            pitch = asin(sp); // -pi/2< p < pi/2
            
            roll = atan2(sp * R(0,1) + R(1,2),  // -cp*cp*sr*cy
                         sp * R(0,2) - R(1,1)); // -cp*cp*cr*cy
            
            if (R(0,0) > 0.0) { // cy > 0
                (roll < 0.0) ? (roll += PI) : (roll -= PI);
            }
            const double sr = sin(roll);
            const double cr = cos(roll);
            if(sp > 0.0){
                yaw = atan2(sr * R(1,1) + cr * R(1,2), //sy*sp
                            sr * R(0,1) + cr * R(0,2));//cy*sp
            } else {
                yaw = atan2(-sr * R(1,1) - cr * R(1,2),
                            -sr * R(0,1) - cr * R(0,2));
            }
        } else {
            yaw = atan2(R(1,0), R(0,0));
            const double sa = sin(yaw);
            const double ca = cos(yaw);
            pitch = atan2(-R(2,0), ca * R(0,0) + sa * R(1,0));
            roll = atan2(sa * R(0,2) - ca * R(1,2), -sa * R(0,1) + ca * R(1,1));
        }
    }

    return Vector3(roll, pitch, yaw);
}


static double getPhase0(double x)
{
    double x0 = fmod(x, 2.0 * M_PI);
    if(x0 >= M_PI){
        x0 -= 2.0 * M_PI;
    } else if(x0 < -M_PI){
        x0 += 2.0 * M_PI;
    }
    return x0;
}


static double adjustPhase(double x, double x_ref)
{
    double adjusted = x;
    double diff = x_ref - x;
    if(diff > 0.0){
        if(diff > M_PI){
            double n = ceil((diff - M_PI) / (2.0 * M_PI));
            adjusted += n * 2.0 * M_PI;
        }
    } else {
        if(diff < -M_PI){
            double n = floor((diff + M_PI) / (2.0 * M_PI));
            adjusted += n * 2.0 * M_PI;
        }
    }
    return adjusted;
}


static double calcPhasedAngleDistance(double x, double y)
{
    return fabs(y - adjustPhase(x, y));
}


Vector3 rpyFromRot(const Matrix3& R, const Vector3& prev)
{
    const double epsilon = 1.0e-6;
    double roll, pitch, yaw;

    Vector3 prev0;
    for(int i=0; i < 3; ++i){
        prev0[i] = getPhase0(prev[i]);
    }
    
    double a = sqrt(R(2,1) * R(2,1) + R(2,2) * R(2,2));

    double p1 = atan2(-R(2,0), a);
    double p2 = atan2(-R(2,0), -a);
    bool use_p1;
    
    if(calcPhasedAngleDistance(p1, prev0.y()) <= calcPhasedAngleDistance(p2, prev0.y())){
        pitch = p1;
        use_p1 = true;
    } else {
        pitch = p2;
        use_p1 = false;
    }
    if(p1 > M_PI / 2.0 - epsilon){
        double b = atan2(-R(1, 2), R(1, 1));
        roll = 0.5 * (prev0.x() + prev0.z() + b);
        yaw = roll - b;

    } else if(p1 < -M_PI / 2.0 + epsilon){
        double b = atan2(-R(1, 2), R(1, 1));
        roll = -0.5 * (prev0.z() - prev0.x() - b);
        yaw = b - roll;
    } else {
        double cy = cos(pitch);
        roll = atan2(R(2,1) / cy, R(2,2) / cy);
        yaw = atan2(R(1,0) / cy, R(0,0) / cy);
        if(!use_p1){
            roll = -roll;
            yaw = -yaw;
        }
    }

    // find the phases closest to the previous position
    roll = adjustPhase(roll, prev.x());
    pitch = adjustPhase(pitch, prev.y());
    yaw = adjustPhase(yaw, prev.z());

    return Vector3(roll, pitch, yaw);
}


Vector3 omegaFromRot(const Matrix3& R)
{
    double alpha = (R(0,0) + R(1,1) + R(2,2) - 1.0) / 2.0;

    if(fabs(alpha - 1.0) < 1.0e-6) {   //th=0,2PI;
        return Vector3::Zero();

    } else {
        double th = acos(alpha);
        double s = sin(th);

        if (s < std::numeric_limits<double>::epsilon()) {   //th=PI
            return Vector3( sqrt((R(0,0)+1)*0.5)*th, sqrt((R(1,1)+1)*0.5)*th, sqrt((R(2,2)+1)*0.5)*th );
        }

        double k = -0.5 * th / s;

        return Vector3((R(1,2) - R(2,1)) * k,
                       (R(2,0) - R(0,2)) * k,
                       (R(0,1) - R(1,0)) * k);
    }
}


std::string str(const Vector3& v)
{
    return fmt::format("{0} {1} {2}", v[0], v[1], v[2]);
}


std::string str(const Vector3f& v)
{
    return fmt::format("{0} {1} {2}", v[0], v[1], v[2]);
}


std::string str(const Vector2& v)
{
    return fmt::format("{0} {1}", v[0], v[1]);
}


std::string str(const AngleAxis& a)
{
    return fmt::format("{0} {1}", str(a.axis()), a.angle());
}


template<class VectorType>
static bool toVector3_(const std::string& s, VectorType& out_v)
{
    const char* nptr = s.c_str();
    char* endptr;
    for(int i=0; i < 3; ++i){
        out_v[i] = strtod(nptr, &endptr);
        if(endptr == nptr){
            return false;
        }
        nptr = endptr;
        while(isspace(*nptr)){
            nptr++;
        }
        if(*nptr == ','){
            nptr++;
        }
    }
    return true;
}    


bool toVector3(const std::string& s, Vector3& out_v)
{
    return toVector3_(s, out_v);
}


bool toVector3(const std::string& s, Vector3f& out_v)
{
    return toVector3_(s, out_v);
}


void normalizeRotation(Matrix3& R)
{
    Matrix3::ColXpr x = R.col(0);
    Matrix3::ColXpr y = R.col(1);
    Matrix3::ColXpr z = R.col(2);
    x.normalize();
    z = x.cross(y).normalized();
    y = z.cross(x);
}


void normalizeRotation(Position& T)
{
    typedef Position::LinearPart::ColXpr ColXpr;
    Position::LinearPart R = T.linear();
    ColXpr x = R.col(0);
    ColXpr y = R.col(1);
    ColXpr z = R.col(2);
    x.normalize();
    z = x.cross(y).normalized();
    y = z.cross(x);
}


void normalizeRotation(Affine3& T)
{
    typedef Affine3::LinearPart::ColXpr ColXpr;
    Affine3::LinearPart R = T.linear();
    ColXpr x = R.col(0);
    ColXpr y = R.col(1);
    ColXpr z = R.col(2);
    x.normalize();
    z = x.cross(y).normalized();
    y = z.cross(x);
}

}
