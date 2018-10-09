/**
   @author Japan Atomic Energy Agency
*/


#include "QuadcopterControllerRTC.h"
#include <cnoid/Joystick>
#include <cmath>

using namespace std;
using namespace cnoid;

namespace {

const double timeStep = 0.005;
const int rotorAxis[] = {
    Joystick::L_STICK_V_AXIS,
    Joystick::R_STICK_H_AXIS, Joystick::R_STICK_V_AXIS,
    Joystick::L_STICK_H_AXIS };
const int cameraAxis = Joystick::DIRECTIONAL_PAD_V_AXIS;
const int powerButton = Joystick::A_BUTTON;

const double sign[4][4] = {
    { 1.0, -1.0, -1.0,  1.0 },
    { 1.0,  1.0, -1.0, -1.0 },
    { 1.0,  1.0,  1.0,  1.0 },
    { 1.0, -1.0,  1.0, -1.0 }
};
const double dir[] = { 1.0, -1.0, 1.0, -1.0 };

static const double KP[] = { 0.4, 0.4, 0.4, 1.0 };
static const double KD[] = { 0.02, 1.0, 1.0, 0.05 };
const double RATE[] = { -1.0, 0.1, -0.1, -0.5 };

}

static const char* multicoptercontrollerrtc_spec[] =
  {
    "implementation_id", "QuadcopterControllerRTC",
    "type_name",         "QuadcopterControllerRTC",
    "description",       "ModuleDescription",
    "version",           "1.0.0",
    "vendor",            "JAEA",
    "category",          "Generic",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    ""
  };

QuadcopterControllerRTC::QuadcopterControllerRTC(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_axesIn("axes", m_axes),
    m_buttonsIn("buttons", m_buttons),
    m_zrpyIn("zrpy", m_zrpy),
    m_qIn("q", m_q),
    m_forceOut("force", m_force),
    m_torqueOut("torque", m_torque),
    m_uOut("u", m_u)

{
}

QuadcopterControllerRTC::~QuadcopterControllerRTC()
{
}



RTC::ReturnCode_t QuadcopterControllerRTC::onInitialize()
{
  addInPort("axes", m_axesIn);
  addInPort("buttons", m_buttonsIn);
  addInPort("zrpy", m_zrpyIn);
  addInPort("q", m_qIn);

  // Set OutPort buffer
  addOutPort("force", m_forceOut);
  addOutPort("torque", m_torqueOut);
  addOutPort("u", m_uOut);
  
  return RTC::RTC_OK;
}

RTC::ReturnCode_t QuadcopterControllerRTC::onActivated(RTC::UniqueId ec_id)
{
    if(m_zrpyIn.isNew()) {
        m_zrpyIn.read();
        if(m_zrpy.data.length() >= 4) {
            for(int i = 0; i < 4; i++) {
                zrpyref[i] = zrpyprev[i] = m_zrpy.data[i];
            }
        }
    }

    if(m_qIn.isNew()) {
        m_qIn.read();
        if(m_q.data.length() >= 1) {
            qref = qprev = m_q.data[0];
        }
    }

    m_force.data.length(4);
    m_torque.data.length(4);
    m_u.data.length(1);

    for(int i = 0; i < 4; i++) {
        m_force.data[i] = 0.0;
        m_torque.data[i] = 0.0;

        dzrpyref[i] = dzrpyprev[i] = 0.0;
        f[i] = 0.0;
    }

    m_u.data[0] = 0.0;

    lastAxes.clear();
    lastAxes.resize(8, 0.0f);

    rotorswitch = false;
    power = powerprev = false;

  return RTC::RTC_OK;
}


RTC::ReturnCode_t QuadcopterControllerRTC::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t QuadcopterControllerRTC::onExecute(RTC::UniqueId ec_id)
{
    if(m_axesIn.isNew()) {
        m_axesIn.read();
        int n = std::min((int)m_axes.data.length(), (int)lastAxes.size());
        for(int i = 0; i < n; i++) {
            lastAxes[i] = m_axes.data[i];
        }
    }

    if(m_buttonsIn.isNew()) {
        m_buttonsIn.read();
        power = m_buttons.data[powerButton];
        if(power == false) {
            powerprev = false;
        }
        else if((power == true) && (powerprev != true)) {
            rotorswitch = !rotorswitch;
            powerprev = true;
        }
    }

    //control rotor
    double cc = cos(m_zrpy.data[1]) * cos(m_zrpy.data[2]);
    double gfcoef = 1.0 * 9.80665 / 4 / cc ;

    double zrpy[4];

    if(m_zrpyIn.isNew()) {
        m_zrpyIn.read();
        if(m_zrpy.data.length() >= 4) {
            for(int i = 0; i < 4; i++) {
                zrpy[i] = m_zrpy.data[i];
            }
        }
    }

    if(rotorswitch) {
        for(int i = 0; i < 4; i++) {
            double dzrpy = (zrpy[i] - zrpyprev[i]) / timeStep;
            if(fabs(dzrpy) > (M_PI / timeStep)) {
                dzrpy = 0.0;
            }
            double ddzrpy = (dzrpy - dzrpyprev[i]) / timeStep;
            double pos = lastAxes[rotorAxis[i]];

            if((i == 0) || (i == 3)) {
                if(fabs(pos) > 0.25) {
                    dzrpyref[i] = RATE[i] * pos;
                }
                else {
                    dzrpyref[i] = 0.0;
                }
                f[i] = KP[i] * (dzrpyref[i] - dzrpy) + KD[i] * (0.0 - ddzrpy);
            }
            else {
                if(fabs(pos) > 0.25) {
                    zrpyref[i] = RATE[i] * pos;
                }
                else {
                    zrpyref[i] = 0.0;
                }
                f[i] = KP[i] * (zrpyref[i] - zrpy[i]) + KD[i] * (0.0 - dzrpy);
            }
            zrpyprev[i] = zrpy[i];
            dzrpyprev[i] = dzrpy;
        }
        for(int i = 0; i < 4; i++) {
            double force = 0.0;
            force += gfcoef;
            force += sign[i][0] * f[0];
            force += sign[i][1] * f[1];
            force += sign[i][2] * f[2];
            force += sign[i][3] * f[3];
            m_force.data[i] = force;
            m_torque.data[i] = dir[i] * force;
        }
    }
    else {
        for(int i = 0; i < 4; i++) {
            m_force.data[i] = 0.0;
            m_torque.data[i] = 0.0;
        }
    }

    m_forceOut.write();
    m_torqueOut.write();

    //control camera
    double q = qprev;
    if(m_qIn.isNew()) {
        m_qIn.read();
        if(m_q.data.length() >= 1) {
            q = m_q.data[0];
        }
    }

    static const double P = 0.00002;
    static const double D = 0.00004;
    double dq = (q - qprev) / timeStep;
    double pos = lastAxes[cameraAxis] * -1.0;
    double dqref = 0.0;
    if(fabs(pos) > 0.25) {
        double deltaq = 0.002 * pos;
        qref += deltaq;
        if(qref > 0) {
            qref = 0.0;
        }
        else if(qref < -M_PI) {
            qref = -M_PI;
        }
        dqref = deltaq / timeStep;
    }
    m_u.data[0] = P * (qref - q) + D * (dqref - dq);
    qprev = q;
    m_uOut.write();

  return RTC::RTC_OK;
}

extern "C"
{
 
  void QuadcopterControllerRTCInit(RTC::Manager* manager)
  {
    coil::Properties profile(multicoptercontrollerrtc_spec);
    manager->registerFactory(profile,
                             RTC::Create<QuadcopterControllerRTC>,
                             RTC::Delete<QuadcopterControllerRTC>);
  }
  
};


