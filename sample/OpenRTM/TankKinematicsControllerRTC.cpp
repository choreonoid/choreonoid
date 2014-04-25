/**
   @author Shizuko Hattori
*/

#include "TankKinematicsControllerRTC.h"

using namespace std;
using namespace cnoid;

namespace {

const double TIMESTEP = 0.001;
const double TANK_POS_Z = 0.1;

const char* samplepd_spec[] =
{
    "implementation_id", "TankKinematicsControllerRTC",
    "type_name",         "TankKinematicsControllerRTC",
    "description",       "Tank Kinematics Controller component",
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


TankKinematicsControllerRTC::TankKinematicsControllerRTC(RTC::Manager* manager)
    : RTC::DataFlowComponentBase(manager),
      m_angleOut("q", m_angle),
      m_rootOut("root", m_root)
{

}


TankKinematicsControllerRTC::~TankKinematicsControllerRTC()
{

}


RTC::ReturnCode_t TankKinematicsControllerRTC::onInitialize()
{
    // Set OutPort buffer
    addOutPort("q", m_angleOut);
    addOutPort("root", m_rootOut);

    return RTC::RTC_OK;
}


RTC::ReturnCode_t TankKinematicsControllerRTC::onActivated(RTC::UniqueId ec_id)
{
    time = 0.0;

    VectorXd data0(5);   // x, y, yaw of root, yaw, pitch of CANNON
    data0 << -0.8, 2.4, radian(-90.0), radian(0.0), radian(0.0);
    VectorXd data1(5);
    data1 << -0.8, 0.8, radian(-90.0), radian(0.0), radian(0.0);
    VectorXd data2(5);
    data2 << -0.5, 0.6, radian(0.0), radian(0.0), radian(0.0);
    VectorXd data3(5);
    data3 << -0.5, 0.6, radian(0.0), radian(-45.0), radian(0.0);
    VectorXd data4(5);
    data4 << -0.5, 0.6, radian(0.0), radian(45.0), radian(0.0);
    VectorXd data5(5);
    data5 << -0.5, 0.6, radian(0.0), radian(0.0), radian(-30.0);
    VectorXd data6(5);
    data6 << -0.5, 0.6, radian(0.0), radian(0.0), radian(30.0);
    VectorXd data7(5);
    data7 << -0.8, 0.6, radian(0.0), radian(0.0), radian(0.0);
    VectorXd data8(5);
    data8 << -0.8, 2.4, radian(90.0), radian(0.0), radian(0.0);

    interpolator.clear();
    interpolator.appendSample(0.0, data0);
    interpolator.appendSample(3.0, data1);
    interpolator.appendSample(6.0, data2);
    interpolator.appendSample(9.0, data3);
    interpolator.appendSample(12.0, data4);
    interpolator.appendSample(15.0, data5);
    interpolator.appendSample(18.0, data6);
    interpolator.appendSample(21.0, data7);
    interpolator.appendSample(24.0, data8);
    interpolator.update();

    m_angle.data.length(2);
    m_root.data.length(12);

    return RTC::RTC_OK;
}


RTC::ReturnCode_t TankKinematicsControllerRTC::onDeactivated(RTC::UniqueId ec_id)
{
    return RTC::RTC_OK;
}


RTC::ReturnCode_t TankKinematicsControllerRTC::onExecute(RTC::UniqueId ec_id)
{
    VectorXd data(5);
    data = interpolator.interpolate(time);
    Matrix3 R = rotFromRpy(0.0, 0.0, data[2]);

    m_root.data[0] = data[0];
    m_root.data[1] = data[1];
    m_root.data[2] = TANK_POS_Z;
    m_root.data[3] = R(0,0);
    m_root.data[4] = R(0,1);
    m_root.data[5] = R(0,2);
    m_root.data[6] = R(1,0);
    m_root.data[7] = R(1,1);
    m_root.data[8] = R(1,2);
    m_root.data[9] = R(2,0);
    m_root.data[10] = R(2,1);
    m_root.data[11] = R(2,2);
    
    m_angle.data[0] = data[3];
    m_angle.data[1] = data[4];
    
    m_rootOut.write();
    m_angleOut.write();

    time += TIMESTEP;

    return RTC::RTC_OK;
}


extern "C"
{

    DLL_EXPORT void TankKinematicsControllerRTCInit(RTC::Manager* manager)
    {
        coil::Properties profile(samplepd_spec);
        manager->registerFactory(profile,
                                 RTC::Create<TankKinematicsControllerRTC>,
                                 RTC::Delete<TankKinematicsControllerRTC>);
    }

};
