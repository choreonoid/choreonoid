/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_OPENRTM_BODY_IO_VRC_H
#define CNOID_OPENRTM_BODY_IO_VRC_H

#include <cnoid/Body>
#include <cnoid/ControllerItem>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include "exportdecl.h"

namespace cnoid {

class Body;

class CNOID_EXPORT BodyIoRTC : public RTC::DataFlowComponentBase
{
public:
    BodyIoRTC(RTC::Manager* manager);
    ~BodyIoRTC();

    virtual RTC::ReturnCode_t onInitialize(Body* body);
    
    virtual bool initializeSimulation(ControllerItemIO* io);
    virtual bool startSimulation();
    virtual void inputFromSimulator();
    virtual void outputToSimulator();
    virtual void stopSimulation();
};

}

#endif
