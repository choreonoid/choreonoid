/**
   @author Japan Atomic Energy Agency
*/

#pragma once
#include "FFCalc_Common.h"

namespace Multicopter {
namespace FFCalc {

void calcFluidForce (
    const Eigen::Vector3d& gravity,
    const FluidEnvironment& fluidEnv,
    const cnoid::Link& link,
    const LinkAttribute& linkAttr,
    const LinkState& linkState,
    const std::vector<LinkTriangleAttribute>& triAttrAry,
    LinkForce* pLinkForce);

}}
