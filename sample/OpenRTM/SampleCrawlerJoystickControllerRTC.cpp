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
      m_axesIn("axes", m_axes),
      m_torqueOut("u", m_torque)
{ 

}

SampleCrawlerJoystickControllerRTC::~SampleCrawlerJoystickControllerRTC()
{

}


RTC::ReturnCode_t SampleCrawlerJoystickControllerRTC::onInitialize()
{
    // Set InPort buffers
    addInPort("axes", m_axesIn);
  
    // Set OutPort buffer
    addOutPort("u", m_torqueOut);

    return RTC::RTC_OK;
}


RTC::ReturnCode_t SampleCrawlerJoystickControllerRTC::onActivated(RTC::UniqueId ec_id)
{
    m_torque.data.length(2);
    m_torque.data[0] = 0.0;
    m_torque.data[1] = 0.0;

    return RTC::RTC_OK;
}


RTC::ReturnCode_t SampleCrawlerJoystickControllerRTC::onDeactivated(RTC::UniqueId ec_id)
{
    return RTC::RTC_OK;
}


RTC::ReturnCode_t SampleCrawlerJoystickControllerRTC::onExecute(RTC::UniqueId ec_id)
{
    if(m_axesIn.isNew()){
        m_axesIn.read();
        m_torque.data[0] = -2.0 * m_axes.data[1] + m_axes.data[0];
        m_torque.data[1] = -2.0 * m_axes.data[1] - m_axes.data[0];
    }
    m_torqueOut.write();

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

