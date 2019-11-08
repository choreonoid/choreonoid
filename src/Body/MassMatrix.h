/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_MASS_MATRIX_H
#define CNOID_BODY_MASS_MATRIX_H

#include "Body.h"
#include "exportdecl.h"

namespace cnoid {

CNOID_EXPORT void calcMassMatrix(Body* body, MatrixXd& out_M);
CNOID_EXPORT void calcMassMatrix(Body* body, const Vector3& g, MatrixXd& out_M, VectorXd& out_b);

}

#endif
