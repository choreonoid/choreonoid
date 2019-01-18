/**
   \file
   \author Shizuko Hattori
*/

#ifndef CNOID_OPENHRP_MODELLOADER_UTIL_H
#define CNOID_OPENHRP_MODELLOADER_UTIL_H

#include <cnoid/corba/OpenHRPModelLoader/ModelLoader.hh>
#include <cnoid/EigenUtil>

using namespace OpenHRP;

namespace cnoid{
namespace openHRPModelloader {

template <typename T, typename S> void setVector3( const T& v, S& out){
    out[0] = v(0);
    out[1] = v(1);
    out[2] = v(2);
}
void setVector3(const Vector3f& v, float* out);
void setAngleAxis(const AngleAxis& a, DblArray4& out);
void setMatrix3(const Matrix3& m, DblArray4& out);
void setMatrix3(const Matrix3& m, DblArray9& out);
void setTransformMatrix(const Matrix3& m, const Vector3& v, DblArray12& out);
void setTransformMatrix(const Position& m, DblArray12& out);
template <typename T> void setFloatSequence(const T& v, OpenHRP::FloatSequence& out){
    out.length(v.size());
    for(int i=0; i<v.size(); i++){
        if(v[i] > std::numeric_limits<float>::max()){
            out[i] = std::numeric_limits<float>::max();
        }else if(v[i] < -std::numeric_limits<float>::max()){
            out[i] = -std::numeric_limits<float>::max();
        }else{
            out[i] = v[i];
        }
    }
}

template <typename T, typename S> void setSequence(T& v, S& out){
    out.length(v.size());
    for(int i=0; i<v.size(); i++){
        out[i] = v[i];
    }
}


}
}

#endif
