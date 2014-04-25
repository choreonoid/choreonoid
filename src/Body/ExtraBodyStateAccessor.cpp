/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "ExtraBodyStateAccessor.h"

using namespace cnoid;

int ExtraBodyStateAccessor::elementSizes[] = {
    1,  // BOOL
    1,  // INT
    1,  // DOUBLE
    1,  // ANGLE
    1,  // STRING
    3,  // VECTOR3
    -1, // VECTORX
    0   // NONE
};

ExtraBodyStateAccessor::ExtraBodyStateAccessor()
{

}


ExtraBodyStateAccessor::ExtraBodyStateAccessor(const ExtraBodyStateAccessor& org)
{
    // sigStateChanged_ shouldn't be copied.
}


ExtraBodyStateAccessor::~ExtraBodyStateAccessor()
{

}


bool ExtraBodyStateAccessor::setState(const std::vector<Value>& state) const
{
    return false;
}


bool ExtraBodyStateAccessor::setJointState(const Array2D<Value>& jointState) const
{
    return false;
}



