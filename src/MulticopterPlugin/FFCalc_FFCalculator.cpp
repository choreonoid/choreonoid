/**
   @author Japan Atomic Energy Agency
*/

#include "MulticopterPluginHeader.h"

namespace Multicopter {
namespace FFCalc {


const double FFCalculator::TINY_VELOCITY = 1.0e-12;

FFCalculator::FFCalculator (
    const Vector3& gravity,
    const FluidEnvironment& fluidEnv,
    const FluidEnvironment::FluidValue& fluidEnvAll,
    const cnoid::Link&      link,
    const LinkAttribute&    linkAttr,
    const LinkState&        linkState,
    const std::vector<LinkTriangleAttribute>& triAttrAry)
        : _gravity(gravity), _fluidEnv(fluidEnv),_fluidEnvAll(fluidEnvAll),_link(link),
          _linkAttr(linkAttr), _linkState(linkState), _triAttrAry(triAttrAry)
{

    const double mass = _link.mass();

    const double density = _linkAttr.density();

    _linkVolume = mass / density;

    return;
}

void FFCalculator::calcBuoyancy(LinkForce* pLinkForce)
{

    const Vector3 centerOfBuoyancy = _linkState.toGlobalPosition (_linkAttr.centerOfBuoyancy());

    FluidEnvironment::FluidValue fluid;
    bool inBounds = _fluidEnv.get (centerOfBuoyancy, fluid);

    if ( inBounds==false  && _fluidEnvAll.isFluid ==false)

        return;

    Vector3 force;
    if(inBounds==true){
        if(fluid.isFluid ==true)force = - fluid.density * _linkVolume * _gravity;
        else return;
    }else if(_fluidEnvAll.isFluid == true){
        force = - _fluidEnvAll.density * _linkVolume * _gravity;
    }else return;
    pLinkForce->addForce (force, centerOfBuoyancy);
    return;
}

void FFCalculator::calcAddMass (LinkForce* pLinkForceT, LinkForce* pLinkForceR)
{

    const Vector3& centerOfMass = _link.R()*_link.c()+_link.p();

    FluidEnvironment::FluidValue fluid;
    bool inBounds = _fluidEnv.get (centerOfMass, fluid);
    if ( inBounds==false  && _fluidEnvAll.isFluid ==false)

        return;

    const double addMassCoef = _linkAttr.additionalMassCoef();

    const Matrix3 addIMatrix   = _linkState.toGlobalMatrix (_linkAttr.additionalInertiaMatrix());

    const Vector3 accTra = _linkState.translationalAccelerationAt (centerOfMass);

    Vector3 accRot = _linkState.rotationalAcceleration();

    const double timeConst = 0.01;

    Vector3 force;

    if(inBounds==true){
        if(fluid.isFluid ==true){
            force = - fluid.density * addMassCoef * _linkVolume * accTra;
        }else return;
    }else if(_fluidEnvAll.isFluid == true){
        force = - _fluidEnvAll.density * addMassCoef * _linkVolume * accTra;
    }else return;

    pLinkForceT->addForce (force, centerOfMass);

    Vector3 moment= - addIMatrix * accRot;

    pLinkForceR->addMoment (moment);

    return;
}

void FFCalculator::calcSurfaceGeneral(LinkForce* pLinkForceN, LinkForce* pLinkForceT,int degreeNumber)
{

    const int numIP = degreeNumber;

    for (const auto& triAttr : _triAttrAry)
    {
        const GaussTriangle3d tri (triAttr.triangle(), _linkState.trans());

        for (int iIP=0; iIP<numIP; ++iIP)
        {

            const double cutCoef = triAttr.cutoffCoefficient(iIP);
            if (cutCoef < 1.0e-12)
                continue;

            const Vector3 posIP = tri.getGaussPoint(iIP,numIP);

            FluidEnvironment::FluidValue fluid;
            bool inBounds = _fluidEnv.get (posIP, fluid);

            if ( inBounds==false  && _fluidEnvAll.isFluid ==false)
                continue;

            if(inBounds==true){
                if(fluid.isFluid ==true){
                }else continue;
            }else if(_fluidEnvAll.isFluid == true){
                fluid=_fluidEnvAll;
            }else continue;

            const double coefIP = cutCoef * tri.getGaussWeight(iIP,numIP) * tri.area();

            double velPerp;
            double velPara;
            Vector3 vePara;

            Vector3 velRelative = fluid.velocity - _linkState.translationalVelocityAt(posIP);
            _calcVectorDecomp (velRelative, -tri.normal(), &velPerp, &velPara, &vePara);

            if (velPerp >= TINY_VELOCITY)
            {
                double pressure = 0.5 * fluid.density * std::pow (std::max (0.0, velPerp), 2);
                Vector3 force = -tri.normal() * pressure;
                pLinkForceN->addForce (coefIP * force, posIP);
            }

            if (velPara >= TINY_VELOCITY)
            {

                double repLength = std::pow (_linkVolume, 1.0/3.0);

                double coefReynolds = fluid.density * velPara * repLength / fluid.viscosity;

                if (coefReynolds < 4.0e5)
                {
                    Vector3 forceT = 0.664 * velPara * vePara *
                                     std::sqrt (fluid.viscosity * fluid.density * velPara / repLength);
                    pLinkForceT->addForce (coefIP * forceT, posIP);
                }
                else
                {
                    double coefResist = 0.455 / std::pow (std::log10(coefReynolds), 2.58) - 1700.0 / coefReynolds;
                    if (coefReynolds < 6.0e5)
                        coefResist = std::max (coefResist, 1.328 / std::sqrt(coefReynolds));

                    Vector3 forceT = coefResist * 0.5 * fluid.density * velPara * velPara * vePara;
                    pLinkForceT->addForce (coefIP * forceT, posIP);
                }
            }
        }
    }
    return;
}

void FFCalculator::calcGravity_forDebug (LinkForce* pLinkForce)
{

    const double mass = _link.mass();

    const Vector3 centerOfMass = _link.R()*_link.c()+_link.p();

    const Vector3 force = mass * _gravity;
    pLinkForce->addForce (force, centerOfMass);

    return;
}

void FFCalculator::_calcFrameAxisDirection (
    const Vector3& vecZ, const Vector3& vecXZ,
    Vector3* vex, Vector3* vey, Vector3* vez)
{
    *vez = vecZ / vecZ.norm();
    *vey = vecZ.cross(vecXZ);
    double nrm = (*vey).norm();
    if (std::abs(nrm)<1.0e-12)
        throw std::runtime_error ("vecZ and vecXZ mustn't be parallel.");
    *vey = (*vey) / nrm;
    *vex = (*vey).cross(*vez);
    return;
}

void FFCalculator::_calcVectorDecomp (
    const Vector3& vec, const Vector3& vDirection, double* normDir,
    double* normPlane, Vector3* vePlane)
{
    *normDir   = vec.dot(vDirection);
    *vePlane   = vec - vDirection * (*normDir);
    *normPlane = (*vePlane).norm();

    if (*normDir <= TINY_VELOCITY)
        *normDir = 0.0;

    if (*normPlane <= TINY_VELOCITY)
    {
        *vePlane << 0.0, 0.0, 0.0;
        *normPlane = 0.0;
    }
    else
    {
        *vePlane   = (*vePlane) / *normPlane;
    }

    return;
}


}}
