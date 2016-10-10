/*!
  @brief A component for accessing a joystick control device
*/

#include "JoystickRTC.h"

// Module specification
static const char* joystick_spec[] =
{
    "implementation_id", "JoystickRTC",
    "type_name",         "JoystickRTC",
    "description",       "Access a joystick control device.",
    "version",           "1.0.0",
    "vendor",            "AIST",
    "category",          "HumanInterfaceDevice",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.device", "/dev/input/js0",
    "conf.default.debugLevel", "0",
    ""
};


JoystickRTC::JoystickRTC(RTC::Manager* manager)
    : RTC::DataFlowComponentBase(manager),
      m_axesOut("axes", m_axes),
      m_buttonsOut("buttons", m_buttons),
      m_debugLevel(0)
{
    m_joystick = 0;
}


JoystickRTC::~JoystickRTC()
{
    if(m_joystick){
        delete m_joystick;
    }
}


RTC::ReturnCode_t JoystickRTC::onInitialize()
{
    // Bind variables and configuration variable
    bindParameter("device", m_device, "/dev/input/js0");
    bindParameter("debugLevel", m_debugLevel, "0");

    // Set OutPort buffer
    addOutPort("axes", m_axesOut);
    addOutPort("buttons", m_buttonsOut);
  
    return RTC::RTC_OK;
}


RTC::ReturnCode_t JoystickRTC::onActivated(RTC::UniqueId ec_id)
{
    if(m_debugLevel > 0){
        std::cout << "JoystickRTC::onActivated(" << ec_id << ")" << std::endl;
    }

    if(!m_joystick){
        m_joystick = new cnoid::Joystick(m_device.c_str());
    }
    if(!m_joystick->isReady()){
        std::cerr << "Joystick device(" << m_device << ") is not opened" << std::endl;
        return RTC::RTC_ERROR;  
    } else {
        int n = m_joystick->numAxes();
        m_axes.data.length(n);
        for(int i=0; i < n; ++i){
            double pos = m_joystick->getPosition(i);
            if(fabs(pos) < 0.2){
                pos = 0.0;
            }
            m_axes.data[i] = pos;
        }
        int m = m_joystick->numButtons();
        m_buttons.data.length(m);
        for(int i=0; i < m; ++i){
            m_buttons.data[i] = m_joystick->getButtonState(i);
        }
        return RTC::RTC_OK;
    }
}


RTC::ReturnCode_t JoystickRTC::onDeactivated(RTC::UniqueId ec_id)
{
    if(m_debugLevel > 0){
        std::cout << "JoystickRTC::onDeactivated(" << ec_id << ")" << std::endl;
    }

    if(m_joystick){
        delete m_joystick;
        m_joystick = 0;
    }
    
    return RTC::RTC_OK;
}



RTC::ReturnCode_t JoystickRTC::onExecute(RTC::UniqueId ec_id)
{
    if(m_debugLevel > 0){
        std::cout << "JoystickRTC::onExecute(" << ec_id << ")" << std::endl;
    }
    
    m_joystick->readCurrentState();
    
    if(m_debugLevel > 0){
        std::cout << "axes:";
    }
    const int n = m_joystick->numAxes();
    for(int i=0; i < n; ++i){
        m_axes.data[i] = m_joystick->getPosition(i);
        if(m_debugLevel > 0){
            std::cout << m_axes.data[i];
        }
    }
    if(m_debugLevel > 0){
        std::cout << ", buttons:";
    }
    const int m = m_joystick->numButtons();
    for(int i=0; i < m; ++i){
        m_buttons.data[i] = m_joystick->getButtonState(i);
        if(m_debugLevel > 0){
            std::cout << m_buttons.data[i];
        }
    }
    if(m_debugLevel > 0){
        std::cout << std::endl;
    }

    m_axesOut.write();
    m_buttonsOut.write();

    return RTC::RTC_OK;
}


extern "C"
{
    void JoystickRTCInit(RTC::Manager* manager)
    {
        coil::Properties profile(joystick_spec);
        manager->registerFactory(
            profile, RTC::Create<JoystickRTC>, RTC::Delete<JoystickRTC>);
    }
};
