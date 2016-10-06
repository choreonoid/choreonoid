/**
   @author Shin'ichiro Nakaoka
*/

#include <cnoid/BodyIoRTC>
#include <cnoid/Light>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>

using namespace std;
using namespace cnoid;

namespace {

class TankRTC : public BodyIoRTC
{
public:
    TankRTC(RTC::Manager* manager);
    ~TankRTC();

    virtual RTC::ReturnCode_t onInitialize(Body* body) override;
    virtual bool initializeSimulation(ControllerItemIO* io) override;
    virtual void inputFromSimulator() override;
    virtual void outputToSimulator() override;

    BodyPtr ioBody;
    
    // DataInPort declaration
    RTC::TimedDoubleSeq torques;
    RTC::InPort<RTC::TimedDoubleSeq> torquesIn;

    RTC::TimedDoubleSeq velocities;
    RTC::InPort<RTC::TimedDoubleSeq> velocitiesIn;

    RTC::TimedBooleanSeq lightSwitch;
    RTC::InPort<RTC::TimedBooleanSeq> lightSwitchIn;
    LightPtr light;
    
    // DataOutPort declaration
    RTC::TimedDoubleSeq angles;
    RTC::OutPort<RTC::TimedDoubleSeq> anglesOut;
};

const char* spec[] =
{
    "implementation_id", "TankRTC",
    "type_name",         "TankRTC",
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

TankRTC::TankRTC(RTC::Manager* manager)
    : BodyIoRTC(manager),
      torquesIn("u", torques),
      velocitiesIn("dq", velocities),
      lightSwitchIn("lightSwitch", lightSwitch),
      anglesOut("q", angles)
{

}


TankRTC::~TankRTC()
{

}


RTC::ReturnCode_t TankRTC::onInitialize(Body* body)
{
    // Set InPort buffers
    addInPort("u", torquesIn);
    addInPort("dq", velocitiesIn);
    addInPort("lightSwitch", lightSwitchIn);
    
    // Set OutPort buffer
    addOutPort("q", anglesOut);
    angles.data.length(ioBody->numJoints());

    return RTC::RTC_OK;
}


bool TankRTC::initializeSimulation(ControllerItemIO* io)
{
    ioBody = io->body();
    light = ioBody->findDevice<Light>("MainLight");
}


void TankRTC::inputFromSimulator()
{
    for(int i=0; i < ioBody->numJoints(); ++i){
        angles.data[i] = ioBody->joint(i)->q();
    }
    anglesOut.write();
}


void TankRTC::outputToSimulator()
{
    if(torquesIn.isNew()){
        torquesIn.read();
        for(int i=0; i < torques.data.length(); ++i){
            ioBody->joint(i)->u() = torques.data[i];
        }
    }
    if(velocitiesIn.isNew()){
        velocitiesIn.read();
        for(int i=0; i < velocities.data.length(); ++i){
            ioBody->joint(i)->dq() = velocities.data[i];
        }
    }
    if(light && lightSwitchIn.isNew()){
        lightSwitchIn.read();
        light->on(lightSwitch.data[0]);
        light->notifyStateChange();
    }
}


extern "C"
{
    DLL_EXPORT void TankRTCInit(RTC::Manager* manager)
    {
        coil::Properties profile(spec);
        manager->registerFactory(
            profile, RTC::Create<TankRTC>, RTC::Delete<TankRTC>);
    }
};
