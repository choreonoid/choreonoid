/**
   @author Shizuko Hattori
*/

#ifndef SR1LiftupHGControllerRTC_H
#define SR1LiftupHGControllerRTC_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include "Interpolator.h"
#include <cnoid/Body>

class SR1LiftupHGControllerRTC : public RTC::DataFlowComponentBase
{
public:
    SR1LiftupHGControllerRTC(RTC::Manager* manager);
    ~SR1LiftupHGControllerRTC();

    virtual RTC::ReturnCode_t onInitialize();
    virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
    virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
    virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

protected:
    // DataInPort declaration
    RTC::TimedDoubleSeq m_angle;
    RTC::InPort<RTC::TimedDoubleSeq> m_angleIn;
  
    RTC::TimedDoubleSeq m_torque_in;
    RTC::InPort<RTC::TimedDoubleSeq> m_torqueIn;

    // DataOutPort declaration
    RTC::TimedDoubleSeq m_vel_out;
    RTC::OutPort<RTC::TimedDoubleSeq> m_velOut;
  
private:
    cnoid::BodyPtr body;
    unsigned int n;
    unsigned int rightWrist_id;
    unsigned int leftWrist_id;
    cnoid::Interpolator<cnoid::VectorXd> interpolator;
    cnoid::VectorXd qref, qold, qref_old;
    double time;
    double timeStep;
    int phase;
    double dq_wrist;
    double throwTime;
};

extern "C"
{
    DLL_EXPORT void SR1LiftupHGControllerRTCInit(RTC::Manager* manager);
};

#endif
