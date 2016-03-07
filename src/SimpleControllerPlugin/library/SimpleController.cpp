/**
   @author Shin'ichiro Nakaoka
*/

#include "SimpleController.h"
#include <cnoid/NullOut>
#include <boost/tokenizer.hpp>

using namespace std;
using namespace boost;
using namespace cnoid;

namespace cnoid {

class SimpleControllerImpl
{
public:
    /*
    mutable BodyPtr ioBody;
    double timeStep;
    boost::dynamic_bitset<> jointOutputFlags;
    bool isImmediateMode;
    mutable std::ostream* os;
    std::vector<std::string> options;
    */
};

}
    

SimpleController::SimpleController()
{
    /*
    impl = new SimpleControllerImpl;
    impl->timeStep = 1.0;
    impl->isImmediateMode = false;
    impl->os = &nullout();
    */

    io = 0;
}

/*
SimpleController::SimpleController(const SimpleController& org)
{
    impl = new SimpleControllerImpl;
    impl->timeStep = org.impl->timeStep;
    impl->isImmediateMode = org.impl->isImmediateMode;
    impl->os = org.impl->os;
    impl->options = org.impl->options;
}
*/


SimpleController::~SimpleController()
{

}


bool SimpleController::initialize(SimpleControllerIO* io)
{
    return false;
}


bool SimpleController::initialize()
{
    return false;
}


bool SimpleController::control(SimpleControllerIO* io)
{
    return false;
}


bool SimpleController::control()
{
    return false;
}


void SimpleController::setIO(SimpleControllerIO* io)
{
    this->io = io;
}


void SimpleController::setJointOutput(bool on)
{
    if(on){
        io->setJointOutput(JOINT_TORQUE);
    } else {
        io->setJointOutput(0);
    }
}
    

void SimpleController::setJointOutput(int jointId, bool on)
{
    if(on){
        io->setLinkOutput(io->body()->joint(jointId), JOINT_TORQUE);
    } else {
        io->setLinkOutput(io->body()->joint(jointId), 0);
    }
}


Body* SimpleController::ioBody()
{
    return io->body();
}


double SimpleController::timeStep() const
{
    return io->timeStep();
}


bool SimpleController::isImmediateMode() const
{
    return io->isImmediateMode();
}


std::ostream& SimpleController::os() const
{
    return io->os();
}
