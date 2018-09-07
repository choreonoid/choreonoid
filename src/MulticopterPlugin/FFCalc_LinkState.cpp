/**
   @author Japan Atomic Energy Agency
*/

#include "MulticopterPluginHeader.h"

#include <sstream>
#include <iomanip>
#include <algorithm>

namespace Multicopter {
namespace FFCalc {

void checkAccelerationError (const cnoid::Link& link, const LinkState::State& currentState, const LinkState::State& previousState)
{

    const LinkState::State& s0 = previousState;
    const LinkState::State& s1 = currentState;

    const Vector3 tacc_div = (s1.tvel - s0.tvel) / (s1.time - s0.time);
    double tacc_base = std::max (s1.tacc.norm(), 9.80665e-2);
    if ((tacc_div-s1.tacc).norm() > tacc_base)
    {
        std::ostringstream oss;
        oss << "Invalid translation acceleration is given";
        oss << " at link " << link.name();
        oss << " at time = " << s1.time;
        UtilityImpl::printErrorMessage(oss.str());
    }

    const Vector3 racc_div = (s1.rvel - s0.rvel) / (s1.time - s0.time);
    double racc_base = std::max (s1.racc.norm(), 6.28319e-2);
    if ((racc_div-s1.racc).norm() > racc_base)
    {
        std::ostringstream oss;
        oss << "Invalid rotational acceleration is given";
        oss << " at link " << link.name();
        oss << " at time = " << s1.time;
        UtilityImpl::printErrorMessage(oss.str());
    }
}

void LinkState::_applyLPF (const State& previousState)
{

    const double timeConst = 0.01;

    const LinkState::State& s0 = previousState;
    LinkState::State&       s1 = *(this->_state);

    double deltaTime = s1.time - s0.time;

    if (deltaTime <= 0.0)
        return;

    double tmp = 1.0 - std::exp (std::max (-deltaTime/timeConst, -50.0));

    s1.tacc = s0.tacc + (s1.tacc - s0.tacc) * tmp;
    s1.racc = s0.racc + (s1.racc - s0.racc) * tmp;

    return;
}

void LinkState::update (const double currentTime, const cnoid::Link& link)
{
    std::unique_ptr<State> state_old (std::move (_state));

    _state.reset (new State (currentTime, link));

#ifdef ENABLE_MULTICOPTER_PLUGIN_DEBUG
    checkAccelerationError (link, *(this->_state), *state_old);

#endif

    return;
}

}}
