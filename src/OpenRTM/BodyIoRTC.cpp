/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "BodyIoRTC.h"

using namespace std;
using namespace cnoid;


BodyIoRTC::BodyIoRTC(RTC::Manager* manager)
    : RTC::DataFlowComponentBase(manager)
{

}


BodyIoRTC::~BodyIoRTC()
{

}


bool BodyIoRTC::initializeIO(ControllerIO* io)
{
    return true;
}


RTC::ReturnCode_t  BodyIoRTC::onInitialize(Body* body)
{
    return RTC::UNSUPPORTED;
}


bool BodyIoRTC::initializeSimulation(ControllerIO* io)
{
    return true;
}


bool BodyIoRTC::startSimulation()
{
    return true;
}


void BodyIoRTC::inputFromSimulator()
{

}


void BodyIoRTC::outputToSimulator()
{

}


void BodyIoRTC::stopSimulation()
{

}
