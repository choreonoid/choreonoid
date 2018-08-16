/**
   @author Japan Atomic Energy Agency
*/


#include <cnoid/BodyIoRTC>
#include <cnoid/ConnectionSet>
#include <cnoid/EigenUtil>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/InterfaceDataTypes.hh>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>

#include <cnoid/RotorDevice>

using namespace std;
using namespace cnoid;
using namespace Multicopter;

const string propname[] = { "PROP0", "PROP1", "PROP2", "PROP3" };
const std::string rotorname[] = { "RotorDevice0", "RotorDevice1", "RotorDevice2", "RotorDevice3" };

namespace {

class QuadcopterIoRTC : public BodyIoRTC
{
public:
    QuadcopterIoRTC(RTC::Manager* manager);
    ~QuadcopterIoRTC();

    virtual bool initializeIO(ControllerIO* io) override;
    virtual bool initializeSimulation(ControllerIO* io) override;
    virtual void inputFromSimulator() override;
    virtual void outputToSimulator() override;

    ScopedConnectionSet connections;

    BodyPtr ioBody;
    Link* prop[4];
    Link* cameraT;
    RotorDevice* rotor[4];

    RTC::TimedDoubleSeq m_u;
    RTC::InPort<RTC::TimedDoubleSeq> m_uIn;

    RTC::TimedDoubleSeq m_torque;
    RTC::InPort<RTC::TimedDoubleSeq> m_torqueIn;

    RTC::TimedDoubleSeq m_force;
    RTC::InPort<RTC::TimedDoubleSeq> m_forceIn;
    
    RTC::TimedDoubleSeq m_q;
    RTC::OutPort<RTC::TimedDoubleSeq> m_qOut;

    RTC::TimedDoubleSeq m_zrpy;
    RTC::OutPort<RTC::TimedDoubleSeq> m_zrpyOut;

};

const char* spec[] =
{
    "implementation_id", "QuadcopterIoRTC",
    "type_name",         "QuadcopterIoRTC",
    "description",       "Quadcopter I/O",
    "version",           "1.0",
    "vendor",            "JAEA",
    "category",          "Generic",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    ""
};

}

QuadcopterIoRTC::QuadcopterIoRTC(RTC::Manager* manager)
    : BodyIoRTC(manager),
      m_uIn("u", m_u),
      m_torqueIn("torque", m_torque),
      m_forceIn("force", m_force),
      m_qOut("q", m_q),
      m_zrpyOut("zrpy", m_zrpy)
{

}


QuadcopterIoRTC::~QuadcopterIoRTC()
{
    connections.disconnect();
}


bool QuadcopterIoRTC::initializeIO(ControllerIO* io)
{
    addInPort("u", m_uIn);
    addInPort("torque", m_torqueIn);
    addInPort("force", m_forceIn);
    
    addOutPort("q", m_qOut);
    m_q.data.length(1);
    addOutPort("zrpy", m_zrpyOut);
    m_zrpy.data.length(4);

    return true;
}


bool QuadcopterIoRTC::initializeSimulation(ControllerIO* io)
{
    ioBody = io->body();

    cameraT = ioBody->link("CAMERA_T");
    cameraT->setActuationMode(Link::JOINT_TORQUE);

    for(int i = 0; i < 4; i++) {
        prop[i] = ioBody->link(propname[i]);
        prop[i]->setActuationMode(Link::JOINT_TORQUE);
        rotor[i] = ioBody->findDevice<RotorDevice>(rotorname[i]);
    }

    connections.disconnect();

    return true;
}


void QuadcopterIoRTC::inputFromSimulator()
{
    m_q.data[0] = cameraT->q();
    m_qOut.write();

    Position T = ioBody->rootLink()->position();
    Vector3 p = T.translation();
    Matrix3 r = T.rotation();
    Vector3 R = rpyFromRot(r);

    m_zrpy.data[0] = p[2];

    m_zrpy.data[1] = R[0];
    m_zrpy.data[2] = R[1];
    m_zrpy.data[3] = R[2];

    m_zrpyOut.write();
}


void QuadcopterIoRTC::outputToSimulator()
{
    if(m_uIn.isNew()){
        m_uIn.read();
        if(m_u.data.length() >= 1){
            cameraT->u() = m_u.data[0];
        }
    }
    if(m_torqueIn.isNew()){
        m_torqueIn.read();
        if(m_torque.data.length() >= 4){
            for(int i = 0; i < 4; i++) {
                rotor[i]->setTorque(m_torque.data[i]);
                if(m_torque.data[i] != 0.0) {
                    prop[i]->u() =  m_torque.data[i]* 0.001;
                }
                else {
                    double dq = prop[i]->dq();
                    prop[i]->u() =  0.0005 * (0.0 - dq);
                }
            }
        }
    }
    if(m_forceIn.isNew()){
        m_forceIn.read();
        if(m_force.data.length() >= 4){
            for(int i = 0; i < 4; i++) {
                rotor[i]->setValue(m_force.data[i]);
            }
        }
    }
}

extern "C"
{
    DLL_EXPORT void QuadcopterIoRTCInit(RTC::Manager* manager)
    {
        coil::Properties profile(spec);
        manager->registerFactory(
            profile, RTC::Create<QuadcopterIoRTC>, RTC::Delete<QuadcopterIoRTC>);
    }
};
