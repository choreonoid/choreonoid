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
      anglesIn("q", angles),
      axesIn("axes", axes),
      buttonsIn("buttons", buttons),
      velocitiesOut("dq", velocities),
      torquesOut("u", torques),
      lightSwitchOut("lightSwitch", lightSwitch)
{

}


TankJoystickControllerRTC::~TankJoystickControllerRTC()
{

}


RTC::ReturnCode_t TankJoystickControllerRTC::onInitialize()
{
    // Set InPort buffers
    addInPort("q", anglesIn);
    addInPort("axes", axesIn);
    addInPort("buttons", buttonsIn);
    
    // Set OutPort buffer
    addOutPort("dq", velocitiesOut);
    addOutPort("u", torquesOut);
    addOutPort("lightSwitch", lightSwitchOut);

    return RTC::RTC_OK;
}


RTC::ReturnCode_t TankJoystickControllerRTC::onActivated(RTC::UniqueId ec_id)
{
    // initialize the cannon joints
    if(anglesIn.isNew()){
        anglesIn.read();
    }
    if(angles.data.length() >= 2){
        torques.data.length(2);
        for(int i=0; i < 2; ++i){
            double q = angles.data[i];
            qref[i] = q;
            qprev[i] = q;
            torques.data[i] = 0.0;
        }
    }

    // initialize the crawler joints
    velocities.data.length(2);
    for(int i=0; i < 2; ++i){
        velocities.data[i] = 0.0;
    }
    lightSwitch.data.length(1);
    prevLightButtonState = false;

    return RTC::RTC_OK;
}


RTC::ReturnCode_t TankJoystickControllerRTC::onDeactivated(RTC::UniqueId ec_id)
{
    return RTC::RTC_OK;
}


RTC::ReturnCode_t TankJoystickControllerRTC::onExecute(RTC::UniqueId ec_id)
{
    if(axesIn.isNew()){
        axesIn.read();
    }

    if(axes.data.length() >= 2){
        for(int i=0; i < 2; ++i){
            if(fabs(axes.data[i]) < 0.2){
                axes.data[i] = 0.0;
            }
        }
        velocities.data[0] = -2.0 * axes.data[1] + axes.data[0];
        velocities.data[1] = -2.0 * axes.data[1] - axes.data[0];
    }
    velocitiesOut.write();

    if(anglesIn.isNew()){
        anglesIn.read();
    }
    if(angles.data.length() >= 2 && axes.data.length() >= 5){

        static const double P = 200.0;
        static const double D = 50.0;

        for(int i=0; i < 2; ++i){
            double q = angles.data[i];
            double dq = (q - qprev[i]) / timeStep;
            double dqref = 0.0;
            double command = cannonAxisRatio[i] * axes.data[cannonAxis[i]];
            if(fabs(command) > 0.2){
                double deltaq = command * 0.002;
                qref[i] += deltaq;
                dqref = deltaq / timeStep;
            }
            torques.data[i] = P * (qref[i] - q) + D * (dqref - dq);
            qprev[i] = q;
        }
    }
    torquesOut.write();

    if(buttonsIn.isNew()){
        buttonsIn.read();
        bool lightButtonState = buttons.data[0];
        if(lightButtonState){
            if(!prevLightButtonState){
                isLightOn = !isLightOn;
                lightSwitch.data[0] = isLightOn;
                lightSwitchOut.write();
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
