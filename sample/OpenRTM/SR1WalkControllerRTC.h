/**
   @author Shizuko Hattori
   @author Shin'ichiro Nakaoka
*/

#ifndef SR1WalkControllerRTC_H
#define SR1WalkControllerRTC_H

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <cnoid/MultiValueSeq>
#include <vector>

class SR1WalkControllerRTC : public RTC::DataFlowComponentBase
{
public:
    SR1WalkControllerRTC(RTC::Manager* manager);
    ~SR1WalkControllerRTC();

    virtual RTC::ReturnCode_t onInitialize();
    virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
    virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
    virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);
    
protected:
    // DataInPort declaration
    RTC::TimedDoubleSeq m_angle;
    RTC::InPort<RTC::TimedDoubleSeq> m_angleIn;
    
    // DataOutPort declaration
    RTC::TimedDoubleSeq m_torque;
    RTC::OutPort<RTC::TimedDoubleSeq> m_torqueOut;
    
private:
    std::shared_ptr<cnoid::MultiValueSeq> qseq;
    std::vector<double> q0;
    cnoid::MultiValueSeq::Frame oldFrame;
    int currentFrame;
    double timeStep_;
};

extern "C"
{
    DLL_EXPORT void SR1WalkControllerRTCInit(RTC::Manager* manager);
};

#endif
