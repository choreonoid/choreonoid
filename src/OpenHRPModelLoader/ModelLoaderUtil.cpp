/**
   \file
   \author Shizuko Hattori
*/
#include "ModelLoaderUtil.h"
#include <iostream>

using namespace std;
using namespace cnoid;

namespace cnoid {
namespace openHRPModelloader {

void setVector3(const Vector3f& v, float* out){
    *out = v(0);
    *(out+1) = v(1);
    *(out+2) = v(2);
}


void setAngleAxis(const AngleAxis& a, DblArray4& out){
    const Vector3& v = a.axis();
    out[0] = v(0);
    out[1] = v(1);
    out[2] = v(2);
    out[3] = a.angle();
}


void setMatrix3(const Matrix3& m, DblArray4& out){
    AngleAxis angleAxis(m);
    setAngleAxis(angleAxis, out);
}


void setMatrix3(const Matrix3& m, DblArray9& out){
    for(int k=0, i=0; i<3; i++){
        for(int j=0; j<3; j++){
            out[k++] = m(i,j);
        }
    }
}


void setTransformMatrix(const Matrix3& m, const Vector3& v, DblArray12& out){
    for(int i=0, k=0; i<3; i++){
        for (int j=0; j<3; j++){
            out[k++] = m(i,j);
        }
        out[k++] = v(i);
    }
}


void setTransformMatrix(const Position& m, DblArray12& out){
    for(int p=0, row=0; row < 3; ++row){
        for(int col=0; col < 4; ++col){
            out[p++] = m(row, col);
        }
    }
}

}
}
