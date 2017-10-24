/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_OPENRTM_BODY_IO_RTC_H
#define CNOID_OPENRTM_BODY_IO_RTC_H

#include <cnoid/Body>
#include <cnoid/ControllerIO>
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

    /**
       \note The io object is dedicated to this function and must not be sotred and used
       in other virtual functions called by the system.
    */
    virtual bool initializeIO(ControllerIO* io);

    /**
       \note The io object given to this function is the object managed by a simulator item.
       It is different from the object given to the initializeIO function, which is not
       managed by the simulator item.
    */
    virtual bool initializeSimulation(ControllerIO* io);
    
    virtual bool startSimulation();
    virtual void inputFromSimulator();
    virtual void outputToSimulator();
    virtual void stopSimulation();

    /**
       \deprecated
       Use the initializeIO function.
    */
    virtual RTC::ReturnCode_t onInitialize(Body* body);
    using RTC::DataFlowComponentBase::onInitialize;
};

}

#endif
