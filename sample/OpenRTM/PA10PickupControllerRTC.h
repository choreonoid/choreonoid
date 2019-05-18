/**
   @author Shizuko Hattori
   @author Shin'ichiro Nakaoka
*/

#ifndef PA10PickupControllerRTC_H
#define PA10PickupControllerRTC_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include "Interpolator.h"
#include <cnoid/Body>
#include <cnoid/JointPath>

class PA10PickupControllerRTC : public RTC::DataFlowComponentBase
{
public:
    PA10PickupControllerRTC(RTC::Manager* manager);
    ~PA10PickupControllerRTC();

    virtual RTC::ReturnCode_t onInitialize();
    virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
    virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
    virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

protected:
    // DataInPort declaration
    RTC::TimedDoubleSeq m_angle;
    RTC::InPort<RTC::TimedDoubleSeq> m_angleIn;
  
    // DataOutPort declaration
    RTC::TimedDoubleSeq m_torque_out;
    RTC::OutPort<RTC::TimedDoubleSeq> m_torqueOut;

private:
    cnoid::BodyPtr body;
    int numJoints;
    int rightHand_id;
    int leftHand_id;
    int phase;
    cnoid::Interpolator<cnoid::VectorXd> wristInterpolator;
    cnoid::Interpolator<cnoid::VectorXd> jointInterpolator;
    cnoid::Link* wrist;
    std::shared_ptr<cnoid::JointPath> baseToWrist;
    double dq_hand;
    cnoid::VectorXd qref, qold, qref_old;
    double time;
};

extern "C"
{
    DLL_EXPORT void PA10PickupControllerRTCInit(RTC::Manager* manager);
};

#endif
