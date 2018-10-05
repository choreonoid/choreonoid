/**
   @author Shin'ichiro Nakaoka
*/

#include "SpreaderController.h"
#include <cnoid/EigenUtil>

using namespace std;
using namespace cnoid;

static SpreaderController* instance = nullptr;


SpreaderController* SpreaderController::instance()
{
    return ::instance;
}


SpreaderController::SpreaderController()
{
    ::instance = this;
}


bool SpreaderController::initialize(SimpleControllerIO* io)
{
    this->io = io;

    flangeL = io->body()->link("FLANGE_WEDGE_L");
    flangeR = io->body()->link("FLANGE_WEDGE_R");

    isToSpreadRequested = false;

    if(flangeL && flangeR){
        flangeL->setActuationMode(Link::JOINT_DISPLACEMENT);
        flangeR->setActuationMode(Link::JOINT_DISPLACEMENT);
        io->enableIO(flangeL);
        io->enableIO(flangeR);
        return true;
    }

    return false;
}


bool SpreaderController::control()
{
    double q_target;
    if(isToSpreadRequested){
        double q = std::max(flangeL->q(), flangeR->q());
        q_target = std::min(q + radian(0.8), flangeL->q_upper());
    } else {
        double q = std::min(flangeL->q(), flangeR->q());
        q_target = std::max(q - radian(0.8), 0.0);
    }
    flangeL->q_target() = q_target;
    flangeR->q_target() = q_target; 
        
    return true;
}


void SpreaderController::requestToSpread(bool on)
{
    isToSpreadRequested = on;
}


CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(SpreaderController)
