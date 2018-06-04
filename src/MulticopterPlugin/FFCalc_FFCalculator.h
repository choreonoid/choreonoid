/**
   @author Japan Atomic Energy Agency
*/

#pragma once
#include "FFCalc_Common.h"

namespace Multicopter {
namespace FFCalc {

class FFCalculator
{
private:
    const FluidEnvironment& _fluidEnv;

    const FluidEnvironment::FluidValue& _fluidEnvAll;

    const cnoid::Link&      _link;

    const LinkAttribute&    _linkAttr;

    const LinkState& _linkState;

    const std::vector<LinkTriangleAttribute>& _triAttrAry;

    double _linkVolume;

    const Vector3 _gravity;

    static const double TINY_VELOCITY;



public:

    FFCalculator (
        const Vector3& gravity,
        const FluidEnvironment& fluidEnv,
        const FluidEnvironment::FluidValue& fluidEnvAll,
        const cnoid::Link&      link,
        const LinkAttribute&    linkAttr,
        const LinkState&        linkState,
        const std::vector<LinkTriangleAttribute>& triAttrAry);

    void calcBuoyancy(LinkForce* pLinkForce);

    void calcAddMass (LinkForce* pLinkForce)
    {
        calcAddMass (pLinkForce, pLinkForce);
    }

    void calcAddMass (LinkForce* pLinkForceT, LinkForce* pLinkForceR);

    void calcSurfaceGeneral (LinkForce* pLinkForceN, LinkForce* pLinkForceT,int);

    void calcGravity_forDebug (LinkForce* pLinkForce);

private:

    void _calcFrameAxisDirection (
        const Vector3& vecZ, const Vector3& vecXZ,
        Vector3* vex, Vector3* vey, Vector3* vez);

    void _calcVectorDecomp (
        const Vector3& vec, const Vector3& vDirection, double* normDir,
        double* normPlane, Vector3* vePlane);
};


}}
