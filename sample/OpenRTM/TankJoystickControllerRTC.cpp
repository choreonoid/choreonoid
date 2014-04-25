/**
   @author Shin'ichiro Nakaoka
*/

#include "TankJoystickControllerRTC.h"
#include <cmath>

using namespace std;

namespace {

const double timeStep = 0.001;
const int numTankJoints = 4;
const int cannonJointId = 2;
const int cannonAxis[] = { 3, 4 };
const double cannonAxisRatio[] = { -1.0, 1.0 };

const char* spec[] =
{
    "implementation_id", "TankJoystickControllerRTC",
    "type_name",         "TankJoystickControllerRTC",
    "description",       "Tank Joystick Controller ",
    "version",           "0.1",
    "vendor",            "AIST",
    "category",          "Generic",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    ""
};
}


TankJoystickControllerRTC::TankJoystickControllerRTC(RTC::Manager* manager)
    : RTC::DataFlowComponentBase(manager),
      m_angleIn("q", m_angle),
      m_axesIn("axes", m_axes),
      m_buttonsIn("buttons", m_buttons),
      m_torqueOut("u", m_torque),
      m_lightOut("light", m_light)
{

}


TankJoystickControllerRTC::~TankJoystickControllerRTC()
{

}


RTC::ReturnCode_t TankJoystickControllerRTC::onInitialize()
{
    // Set InPort buffers
    addInPort("q", m_angleIn);
    addInPort("axes", m_axesIn);
    addInPort("buttons", m_buttonsIn);
    
    // Set OutPort buffer
    addOutPort("u", m_torqueOut);
    addOutPort("light", m_lightOut);

    return RTC::RTC_OK;
}


RTC::ReturnCode_t TankJoystickControllerRTC::onActivated(RTC::UniqueId ec_id)
{
    if(m_angleIn.isNew()){
        m_angleIn.read();
    }
    if(m_angle.data.length() >= 4){
        for(int i=0; i < 2; ++i){
            double q = m_angle.data[cannonJointId + i];
            qref[i] = q;
            qprev[i] = q;
        }
    }
    
    m_torque.data.length(numTankJoints);
    for(int i=0; i < numTankJoints; ++i){
        m_torque.data[i] = 0.0;
    }
    m_light.data.length(1);
    prevLightButtonState = false;

    return RTC::RTC_OK;
}


RTC::ReturnCode_t TankJoystickControllerRTC::onDeactivated(RTC::UniqueId ec_id)
{
    return RTC::RTC_OK;
}


RTC::ReturnCode_t TankJoystickControllerRTC::onExecute(RTC::UniqueId ec_id)
{
    if(m_axesIn.isNew()){
        m_axesIn.read();
    }
    if(m_angleIn.isNew()){
        m_angleIn.read();
    }

    if(m_axes.data.length() >= 2){
        for(int i=0; i < 2; ++i){
            if(fabs(m_axes.data[i]) < 0.2){
                m_axes.data[i] = 0.0;
            }
        }
        m_torque.data[0] = -2.0 * m_axes.data[1] + m_axes.data[0];
        m_torque.data[1] = -2.0 * m_axes.data[1] - m_axes.data[0];
    }

    if(m_angle.data.length() >= 4 && m_axes.data.length() >= 5){

        static const double P = 200.0;
        static const double D = 50.0;

        for(int i=0; i < 2; ++i){
            int jointId = cannonJointId + i;
            double q = m_angle.data[jointId];
            double dq = (q - qprev[i]) / timeStep;
            double dqref = 0.0;
            double command = cannonAxisRatio[i] * m_axes.data[cannonAxis[i]];
            if(fabs(command) > 0.2){
                double deltaq = command * 0.002;
                qref[i] += deltaq;
                dqref = deltaq / timeStep;
            }
            m_torque.data[jointId] = P * (qref[i] - q) + D * (dqref - dq);
            qprev[i] = q;
        }
    }
    
    m_torqueOut.write();

    if(m_buttonsIn.isNew()){
        m_buttonsIn.read();
        bool lightButtonState = m_buttons.data[0];
        if(lightButtonState){
            if(!prevLightButtonState){
                isLightOn = !isLightOn;
                m_light.data[0] = isLightOn;
                m_lightOut.write();
            }
        }
        prevLightButtonState = lightButtonState;
    }

    return RTC::RTC_OK;
}


extern "C"
{
    DLL_EXPORT void TankJoystickControllerRTCInit(RTC::Manager* manager)
    {
        coil::Properties profile(spec);
        manager->registerFactory(profile,
                                 RTC::Create<TankJoystickControllerRTC>,
                                 RTC::Delete<TankJoystickControllerRTC>);
    }
};
