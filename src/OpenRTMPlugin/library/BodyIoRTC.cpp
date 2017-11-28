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


RTC::ReturnCode_t  BodyIoRTC::onInitialize(Body* body)
{
    return RTC::RTC_OK;
}


bool BodyIoRTC::initializeSimulation(ControllerItemIO* io)
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

