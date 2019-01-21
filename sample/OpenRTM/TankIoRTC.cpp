/**
   @author Shin'ichiro Nakaoka
*/

#include <cnoid/BodyIoRTC>
#include <cnoid/AccelerationSensor>
#include <cnoid/RateGyroSensor>
#include <cnoid/Light>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>

using namespace std;
using namespace cnoid;

namespace {

class TankIoRTC : public BodyIoRTC
{
public:
    TankIoRTC(RTC::Manager* manager);
    ~TankIoRTC();

    virtual bool initializeIO(ControllerIO* io) override;
    virtual bool initializeSimulation(ControllerIO* io) override;
    virtual void inputFromSimulator() override;
    virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id) override;
    virtual void outputToSimulator() override;

    BodyPtr ioBody;
    Link* turretY;
    Link* turretP;
    Link* trackL;
    Link* trackR;
    AccelerationSensor* accelSensor;
    RateGyroSensor* gyro;
    Light* light;
    
    // DataInPort declaration
    RTC::TimedDoubleSeq torques;
    RTC::InPort<RTC::TimedDoubleSeq> torquesIn;

    RTC::TimedDoubleSeq velocities;
    RTC::InPort<RTC::TimedDoubleSeq> velocitiesIn;

    RTC::TimedBoolean lightSwitch;
    RTC::InPort<RTC::TimedBoolean> lightSwitchIn;
    
    // DataOutPort declaration
    RTC::TimedDoubleSeq angles;
    RTC::OutPort<RTC::TimedDoubleSeq> anglesOut;

    RTC::TimedAcceleration3D accel;
    RTC::OutPort<RTC::TimedAcceleration3D> accelOut;

    RTC::TimedAngularVelocity3D angularVelocity;
    RTC::OutPort<RTC::TimedAngularVelocity3D> angularVelocityOut;
};


const char* spec[] =
{
    "implementation_id", "TankIoRTC",
    "type_name",         "TankIoRTC",
    "description",       "Tank I/O",
    "version",           "1.0",
    "vendor",            "AIST",
    "category",          "Generic",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    ""
};

}

TankIoRTC::TankIoRTC(RTC::Manager* manager)
    : BodyIoRTC(manager),
      torquesIn("u", torques),
      velocitiesIn("dq", velocities),
      lightSwitchIn("light", lightSwitch),
      anglesOut("q", angles),
      accelOut("dv", accel),
      angularVelocityOut("w", angularVelocity)
{

}


TankIoRTC::~TankIoRTC()
{

}


bool TankIoRTC::initializeIO(ControllerIO* io)
{
    // Set InPort buffers
    addInPort("u", torquesIn);
    addInPort("dq", velocitiesIn);
    addInPort("light", lightSwitchIn);
    
    // Set OutPort buffer
    addOutPort("q", anglesOut);
    angles.data.length(2);
    addOutPort("dv", accelOut);
    addOutPort("w", angularVelocityOut);

    return true;
}


bool TankIoRTC::initializeSimulation(ControllerIO* io)
{
    ioBody = io->body();

    turretY = ioBody->link("TURRET_Y");
    turretP = ioBody->link("TURRET_P");
    turretY->setActuationMode(Link::JOINT_TORQUE);
    turretP->setActuationMode(Link::JOINT_TORQUE);

    trackL = ioBody->link("TRACK_L");
    trackR = ioBody->link("TRACK_R");
    trackL->setActuationMode(Link::JOINT_SURFACE_VELOCITY);
    trackR->setActuationMode(Link::JOINT_SURFACE_VELOCITY);

    accelSensor = ioBody->findDevice<AccelerationSensor>("ACCEL_SENSOR");
    gyro = ioBody->findDevice<RateGyroSensor>("GYRO");
    light = ioBody->findDevice<Light>("Light");
    
    lightSwitch.data = light->on();

    return true;
}


void TankIoRTC::inputFromSimulator()
{
    angles.data[0] = turretY->q();
    angles.data[1] = turretP->q();

    auto dv = accelSensor->dv();
    accel.data.ax = dv.x();
    accel.data.ay = dv.y();
    accel.data.az = dv.z();

    auto w = gyro->w();
    angularVelocity.data.avx = w.x();
    angularVelocity.data.avy = w.y();
    angularVelocity.data.avz = w.z();
}


RTC::ReturnCode_t TankIoRTC::onExecute(RTC::UniqueId ec_id)
{
    anglesOut.write();
    accelOut.write();
    
    if(torquesIn.isNew()){
        torquesIn.read();
    }
    if(velocitiesIn.isNew()){
        velocitiesIn.read();
    }
    if(light && lightSwitchIn.isNew()){
        lightSwitchIn.read();
    }
    return RTC::RTC_OK;
}


void TankIoRTC::outputToSimulator()
{
    if(torques.data.length() >= 2){
        turretY->u() = torques.data[0];
        turretP->u() = torques.data[1];
    }
    if(velocities.data.length() >= 2){
        trackL->dq_target() = velocities.data[0];
        trackR->dq_target() = velocities.data[1];
    }
    bool on = lightSwitch.data;
    if(on != light->on()){
        light->on(on);
        light->notifyStateChange();
    }
}


extern "C"
{
    DLL_EXPORT void TankIoRTCInit(RTC::Manager* manager)
    {
        coil::Properties profile(spec);
        manager->registerFactory(
            profile, RTC::Create<TankIoRTC>, RTC::Delete<TankIoRTC>);
    }
};
