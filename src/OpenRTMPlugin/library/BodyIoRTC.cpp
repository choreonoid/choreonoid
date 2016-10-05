/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include"BodyIoRTC.h"

using namespace std;
using namespace cnoid;

BodyIoRTC::BodyIoRTC(RTC::Manager* manager)
    : RTC::DataFlowComponentBase(manager)
{

}


BodyIoRTC::~BodyIoRTC()
{

}


bool BodyIoRTC::initializePorts(Body* body)
{
    return true;
}


bool BodyIoRTC::initializeSimulation(ControllerItemIO* io)
{
    return true;
}
