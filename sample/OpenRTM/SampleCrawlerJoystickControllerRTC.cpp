/**
   @author Shizuko Hattori
   @author Shin'ichiro Nakaoka
*/

#include "SampleCrawlerJoystickControllerRTC.h"

static const char* spec[] =
{
    "implementation_id", "SampleCrawlerJoystickControllerRTC",
    "type_name",         "SampleCrawlerJoystickControllerRTC",
    "description",       "Sample Crawler Joystick Controller component",
    "version",           "0.1",
    "vendor",            "AIST",
    "category",          "Generic",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    ""
};


SampleCrawlerJoystickControllerRTC::SampleCrawlerJoystickControllerRTC(RTC::Manager* manager)
    : RTC::DataFlowComponentBase(manager),
      axesIn("axes", axes),
      velocitiesOut("dq", velocities)
{ 

}

SampleCrawlerJoystickControllerRTC::~SampleCrawlerJoystickControllerRTC()
{

}


RTC::ReturnCode_t SampleCrawlerJoystickControllerRTC::onInitialize()
{
    // Set InPort buffers
    addInPort("axes", axesIn);
  
    // Set OutPort buffer
    addOutPort("dq", velocitiesOut);

    return RTC::RTC_OK;
}


RTC::ReturnCode_t SampleCrawlerJoystickControllerRTC::onActivated(RTC::UniqueId ec_id)
{
    velocities.data.length(2);
    velocities.data[0] = 0.0;
    velocities.data[1] = 0.0;

    return RTC::RTC_OK;
}


RTC::ReturnCode_t SampleCrawlerJoystickControllerRTC::onDeactivated(RTC::UniqueId ec_id)
{
    return RTC::RTC_OK;
}


RTC::ReturnCode_t SampleCrawlerJoystickControllerRTC::onExecute(RTC::UniqueId ec_id)
{
    if(axesIn.isNew()){
        axesIn.read();
        velocities.data[0] = -2.0 * axes.data[1] + axes.data[0];
        velocities.data[1] = -2.0 * axes.data[1] - axes.data[0];
    }
    velocitiesOut.write();

    return RTC::RTC_OK;
}


extern "C"
{
    DLL_EXPORT void SampleCrawlerJoystickControllerRTCInit(RTC::Manager* manager)
    {
        coil::Properties profile(spec);
        manager->registerFactory(profile,
                                 RTC::Create<SampleCrawlerJoystickControllerRTC>,
                                 RTC::Delete<SampleCrawlerJoystickControllerRTC>);
    }

};
