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
       This function is called when the BodyIoRTC is created by a BodyIoRTCItem.
       If there is an associated BodyItem, its body object can be obtained from the io object
       and a set of ports corresponding to the body model can dynamically be created in
       this function.
    */
    virtual bool initializeIO(ControllerIO* io);

    /**
       This function is called when the simulation is initialized.
    */
    virtual bool initializeSimulation(ControllerIO* io);

    /**
       This function is called when the simulation is started.

       \note The simulation is first initialized, and then
       it is started if the initialization is successfull.
    */
    virtual bool startSimulation();

    /**
       This function is called just before every step of the simulation.
       The process of outputting the states corresponding to the out-ports
       is implemented in this function.
    */
    virtual void inputFromSimulator();
    
    /**
       This function is called just after every step of the simulation.
       The process of applying the commands that are input from the in-ports
       is implemented in this function.
    */
    virtual void outputToSimulator();

    /**
       This function is called when the simulation is stopped.
    */
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
