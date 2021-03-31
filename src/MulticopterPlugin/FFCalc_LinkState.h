/**
   @author Japan Atomic Energy Agency
*/

#pragma once
#include "FFCalc_Common.h"

#include <memory>

namespace Multicopter {
namespace FFCalc {

class LinkState
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    struct State
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        double time;
        const Transform3 trans;
        const Vector3 origin;
        Vector3 tvel;
        Vector3 rvel;
        Vector3 tacc;
        Vector3 racc;
        const Vector3 tacc_raw;
        const Vector3 racc_raw;

        State (const double currentTime, const cnoid::Link& link)
            : time(currentTime), trans(link.T()), origin(link.p()),
              tvel(link.v()), rvel(link.w()), tacc(link.dv()), racc(link.dw()),
              tacc_raw(link.dv()), racc_raw(link.dw()) { }
    };

private:
    std::unique_ptr<State> _state;

public:

    LinkState (const double currentTime, const cnoid::Link& link)
        : _state(new State(currentTime, link)) { }

    const double& time() const { return _state->time; }

    const Transform3& trans() const { return _state->trans; }

    Vector3 toGlobalPosition (const Vector3& pos) const
    {
        return _state->trans * pos;
    }

    Vector3 toGlobalDirection (const Vector3& vec) const
    {
        return _state->trans.linear() * vec;
    }

    Matrix3 toGlobalMatrix (const Matrix3& mat) const
    {
        return _state->trans.linear() * mat;
    }

    Vector3 translationalVelocityAt (const Vector3& positionGlobal) const
    {
        return _translationalEntityAt (_state->tvel, _state->rvel, positionGlobal, _state->origin);
    }

    Vector3 translationalVelocityAtLocal (const Vector3& positionLocal) const
    {
        Vector3 positionGlobal = this->toGlobalPosition(positionLocal);
        return _translationalEntityAt (_state->tvel, _state->rvel, positionGlobal, _state->origin);
    }

    const Vector3& rotationalVelocity() const
    {
        return _state->rvel;
    }

    Vector3 translationalAccelerationAt (const Vector3& positionGlobal) const
    {
        return _translationalEntityAt (_state->tacc, _state->racc, positionGlobal, _state->origin);
    }

    Vector3 translationalAccelerationAtLocal (const Vector3& positionLocal) const
    {
        Vector3 positionGlobal = this->toGlobalPosition(positionLocal);
        return _translationalEntityAt (_state->tacc, _state->racc, positionGlobal, _state->origin);
    }

    const Vector3& rotationalAcceleration() const
    {
        return _state->racc;
    }

    Vector3 translationalRawAccelerationAt (const Vector3& positionGlobal) const
    {
        return _translationalEntityAt (_state->tacc_raw, _state->racc_raw, positionGlobal, _state->origin);
    }

    Vector3 translationalRawAccelerationAtLocal (const Vector3& positionLocal) const
    {
        Vector3 positionGlobal = this->toGlobalPosition(positionLocal);
        return _translationalEntityAt (_state->tacc_raw, _state->racc_raw, positionGlobal, _state->origin);
    }

    const Vector3& rotationalRawAcceleration() const
    {
        return _state->racc_raw;
    }

    void update (const double currentTime, const cnoid::Link& link);

    void initialize (const double currentTime, const cnoid::Link& link)
    {
        _state.reset (new State (currentTime, link));
    }



private:

    static Vector3 _translationalEntityAt (const Vector3& tra, const Vector3& rot, const Vector3& pos, const Vector3& origin)
    {
        return tra + rot.cross (pos - origin);
    }

    void _applyLPF (const State& previousState);
};

typedef std::shared_ptr<LinkState> LinkStatePtr;

}}
