/**
   @author Shin'ichiro Nakaoka
*/

#ifndef TankJoystickControllerRTC_H
#define TankJoystickControllerRTC_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
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
    RTC::TimedDoubleSeq angles;
    RTC::InPort<RTC::TimedDoubleSeq> anglesIn;
    
    RTC::Acceleration3D accel;
    RTC::InPort<RTC::Acceleration3D> accelIn;

    RTC::TimedFloatSeq axes;
    RTC::InPort<RTC::TimedFloatSeq> axesIn;
  
    RTC::TimedBooleanSeq buttons;
    RTC::InPort<RTC::TimedBooleanSeq> buttonsIn;

    // DataOutPort declaration
    RTC::TimedDoubleSeq velocities;
    RTC::OutPort<RTC::TimedDoubleSeq> velocitiesOut;

    RTC::TimedDoubleSeq torques;
    RTC::OutPort<RTC::TimedDoubleSeq> torquesOut;
    
    RTC::TimedBooleanSeq lightSwitch;
    RTC::OutPort<RTC::TimedBooleanSeq> lightSwitchOut;
  
private:
    double qref[2];
    double qprev[2];
    std::vector<float> lastAxes;
    bool lastLightButtonState;
    bool isLightOn;
    int lightBlinkCounter;
    int lightBlinkDuration;
};

extern "C"
{
    DLL_EXPORT void TankJoystickControllerRTCInit(RTC::Manager* manager);
};

#endif
