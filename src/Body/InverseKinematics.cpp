#include "InverseKinematics.h"

using namespace cnoid;


InverseKinematics::~InverseKinematics()
{

}


bool InverseKinematics::calcRemainingPartForwardKinematicsForInverseKinematics()
{
    return false;
}


bool InverseKinematics::isBestEffortIkAvailable() const
{
    return false;  // Default: not supported
}


bool InverseKinematics::isBestEffortIkEnabled() const
{
    return false;  // Default: disabled
}


void InverseKinematics::setBestEffortIkEnabled(bool on)
{
    // Default: do nothing (not supported)
}


int InverseKinematics::getDOF() const
{
    return -1;  // -1 indicates unknown/not available
}
