/**
   @author Shin'ichiro Nakaoka
*/

#ifndef TankJoystickControllerRTC_H
#define TankJoystickControllerRTC_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>

class TankJoystickControllerRTC : public RTC::DataFlowComponentBase
{
public:
    TankJoystickControllerRTC(RTC::Manager* manager);
    ~TankJoystickControllerRTC();
    
    virtual RTC::ReturnCode_t onInitialize();
    virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
    virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
    virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);
    
protected:
    // DataInPort declaration
    RTC::TimedDoubleSeq m_angle;
    RTC::InPort<RTC::TimedDoubleSeq> m_angleIn;
    
    RTC::TimedFloatSeq m_axes;
    RTC::InPort<RTC::TimedFloatSeq> m_axesIn;
  
    RTC::TimedBooleanSeq m_buttons;
    RTC::InPort<RTC::TimedBooleanSeq> m_buttonsIn;

    // DataOutPort declaration
    RTC::TimedDoubleSeq m_torque;
    RTC::OutPort<RTC::TimedDoubleSeq> m_torqueOut;

    RTC::TimedBooleanSeq m_light;
    RTC::OutPort<RTC::TimedBooleanSeq> m_lightOut;
  
private:
    double qref[2];
    double qprev[2];
    bool prevLightButtonState;
    bool isLightOn;
};

extern "C"
{
    DLL_EXPORT void TankJoystickControllerRTCInit(RTC::Manager* manager);
};

#endif
