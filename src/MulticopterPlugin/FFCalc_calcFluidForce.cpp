/**
   @author Japan Atomic Energy Agency
*/

#include "MulticopterPluginHeader.h"

namespace Multicopter {
namespace FFCalc {

void calcFluidForce (
    const Vector3& gravity,
    const FluidEnvironment& fluidEnv,
    const FluidEnvironment::FluidValue& fluidEnvAll,
    const cnoid::Link& link,
    const LinkAttribute& linkAttr,
    const LinkState&        linkState,
    const std::vector<LinkTriangleAttribute>& triAttrAry,
    LinkForce* pLinkForce)
{
    FFCalculator ffc (gravity, fluidEnv, fluidEnvAll, link, linkAttr, linkState, triAttrAry);

    ffc.calcBuoyancy(pLinkForce);

    ffc.calcAddMass(pLinkForce);

    ffc.calcSurfaceGeneral (pLinkForce, pLinkForce,4);

    return;
}

}}
