/**
   @author Shizuko Hattori
*/

#ifndef TankKinematicsControllerRTC_H
#define TankKinematicsControllerRTC_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <cnoid/EigenUtil>
#include "Interpolator.h"

class TankKinematicsControllerRTC : public RTC::DataFlowComponentBase
{
public:
    TankKinematicsControllerRTC(RTC::Manager* manager);
    ~TankKinematicsControllerRTC();

    virtual RTC::ReturnCode_t onInitialize();
    virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
    virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
    virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

protected:
    // DataOutPort declaration
    RTC::TimedDoubleSeq m_angle;
    RTC::OutPort<RTC::TimedDoubleSeq> m_angleOut;

    RTC::TimedDoubleSeq m_root;
    RTC::OutPort<RTC::TimedDoubleSeq> m_rootOut;
  
private:
    double time;
    cnoid::Interpolator<cnoid::VectorXd> interpolator;

};

extern "C"
{
    DLL_EXPORT void TankKinematicsControllerRTCInit(RTC::Manager* manager);
};

#endif
