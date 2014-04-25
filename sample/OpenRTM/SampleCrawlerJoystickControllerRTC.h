/**
   @author Shizuko Hattori
   @author Shin'ichiro Nakaoka
*/

#ifndef SampleCrawlerJoystickControllerRTC_H
#define SampleCrawlerJoystickControllerRTC_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>

class SampleCrawlerJoystickControllerRTC : public RTC::DataFlowComponentBase
{
public:
    SampleCrawlerJoystickControllerRTC(RTC::Manager* manager);
    ~SampleCrawlerJoystickControllerRTC();

    virtual RTC::ReturnCode_t onInitialize();
    virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
    virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
    virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

protected:
    // DataInPort declaration
    RTC::TimedFloatSeq m_axes;
    RTC::InPort<RTC::TimedFloatSeq> m_axesIn;
  
    // DataOutPort declaration
    RTC::TimedDoubleSeq m_torque;
    RTC::OutPort<RTC::TimedDoubleSeq> m_torqueOut;
};

extern "C"
{
    DLL_EXPORT void SampleCrawlerJoystickControllerRTCInit(RTC::Manager* manager);
};

#endif
