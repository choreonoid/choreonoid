/**
   @author Shin'ichiro Nakaoka
*/

#include <cnoid/BodyIoRTC>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>

using namespace std;
using namespace cnoid;

namespace {

class RobotIoRTC : public BodyIoRTC
{
public:
    RobotIoRTC(RTC::Manager* manager);
    ~RobotIoRTC();

    virtual bool initializeIO(ControllerIO* io) override;
    virtual bool initializeSimulation(ControllerIO* io) override;
    virtual void inputFromSimulator() override;
    virtual void outputToSimulator() override;

    BodyPtr body;
    
    // DataInPort declaration
    RTC::TimedDoubleSeq torques;
    RTC::InPort<RTC::TimedDoubleSeq> torquesIn;

    // DataOutPort declaration
    RTC::TimedDoubleSeq angles;
    RTC::OutPort<RTC::TimedDoubleSeq> anglesOut;
};

const char* spec[] =
{
    "implementation_id", "RobotIoRTC",
    "type_name",         "RobotIoRTC",
    "description",       "Robot I/O",
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


RobotIoRTC::RobotIoRTC(RTC::Manager* manager)
    : BodyIoRTC(manager),
      torquesIn("u", torques),
      anglesOut("q", angles)
{

}


RobotIoRTC::~RobotIoRTC()
{

}


bool RobotIoRTC::initializeIO(ControllerIO* io)
{
    // Set InPort buffers
    addInPort("u", torquesIn);
    
    // Set OutPort buffer
    addOutPort("q", anglesOut);
    angles.data.length(io->body()->numJoints());

    return true;
}


bool RobotIoRTC::initializeSimulation(ControllerIO* io)
{
    body = io->body();

    for(auto joint : body->joints()){
        if(joint->isRevoluteJoint() || joint->isPrismaticJoint()){
            joint->setActuationMode(Link::JOINT_TORQUE);
        }
    }
    
    return true;
}


void RobotIoRTC::inputFromSimulator()
{
    for(auto joint : body->joints()){
        int index = joint->jointId();
        angles.data[index] = joint->q();
    }
    anglesOut.write();
}


void RobotIoRTC::outputToSimulator()
{
    if(torquesIn.isNew()){
        torquesIn.read();
        int n = torques.data.length();
        for(int i=0; i < n; ++i){
            if(i < body->numJoints()){
                body->joint(i)->u() = torques.data[i];
            }
        }
    }
}


extern "C"
{
    DLL_EXPORT void RobotIoRTCInit(RTC::Manager* manager)
    {
        coil::Properties profile(spec);
        manager->registerFactory(
            profile, RTC::Create<RobotIoRTC>, RTC::Delete<RobotIoRTC>);
    }
};
