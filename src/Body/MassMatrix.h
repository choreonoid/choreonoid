/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_MASS_MATRIX_H_INCLUDED
#define CNOID_BODY_MASS_MATRIX_H_INCLUDED

#include "Body.h"
#include "exportdecl.h"

namespace cnoid {

CNOID_EXPORT void calcMassMatrix(const BodyPtr& body, const Vector3& g, MatrixXd& out_M);
CNOID_EXPORT void calcMassMatrix(const BodyPtr& body, MatrixXd& out_M);

}

#endif
