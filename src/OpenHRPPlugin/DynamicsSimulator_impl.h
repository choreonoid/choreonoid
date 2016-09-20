
#ifndef CNOID_OPENHRP_DYNAMICS_SIMULATOR_IMPL_H_INCLUDED
#define CNOID_OPENHRP_DYNAMICS_SIMULATOR_IMPL_H_INCLUDED

#ifdef OPENHRP_3_0
#include <cnoid/corba/OpenHRP/3.0/DynamicsSimulator.hh>
#elif OPENHRP_3_1
#include <cnoid/corba/OpenHRP/3.1/DynamicsSimulator.hh>
#endif

#include <cnoid/Body>
#include <cnoid/BasicSensors>

namespace OpenHRP {

class DynamicsSimulator_impl : virtual public POA_OpenHRP::DynamicsSimulator,
                               virtual public PortableServer::RefCountServantBase
{
public:
        
    DynamicsSimulator_impl(const cnoid::BodyPtr& body);
    virtual ~DynamicsSimulator_impl();

#ifdef OPENHRP_3_0
    virtual void registerCharacter(const char* name, CharacterInfo_ptr cinfo);
    virtual void registerCollisionCheckPair(const char* char1, const char* name1, const char* char2, const char* name2, ::CORBA::Double staticFriction, ::CORBA::Double slipFriction, const DblSequence6& K, const DblSequence6& C);
    virtual void checkCollision();

#elif OPENHRP_3_1
    virtual void registerCharacter(const char* name, BodyInfo_ptr cinfo);
    virtual void registerCollisionCheckPair(const char* char1, const char* name1, const char* char2, const char* name2, ::CORBA::Double staticFriction, ::CORBA::Double slipFriction, const DblSequence6& K, const DblSequence6& C, ::CORBA::Double culling_thresh);
    virtual void registerIntersectionCheckPair(const char* char1, const char* name1, const char* char2, const char* name2, ::CORBA::Double tolerance);
    virtual ::CORBA::Boolean checkCollision(::CORBA::Boolean checkAll);
    virtual LinkPairSequence* checkIntersection(::CORBA::Boolean checkAll);
    virtual DistanceSequence* checkDistance();
#endif

    virtual void destroy();
    virtual void init(::CORBA::Double timeStep, OpenHRP::DynamicsSimulator::IntegrateMethod integrateOpt, OpenHRP::DynamicsSimulator::SensorOption sensorOpt);
    virtual void registerVirtualLink(const char* char1, const char* link1, const char* char2, const char* link2, const LinkPosition& relTransform, ::CORBA::Short transformDefined, const DblSequence9& constraint, const char* connectionName);
    virtual void getConnectionConstraintForce(const char* characterName, const char* connectionName, DblSequence6_out contactForce);
    virtual void getCharacterSensorValues(const char* characterName, const char* sensorName, DblSequence_out values);
    virtual void initSimulation();
    virtual void stepSimulation();
    virtual void setCharacterLinkData(const char* character, const char* link, OpenHRP::DynamicsSimulator::LinkDataType type, const DblSequence& data);
    virtual void getCharacterLinkData(const char* characterName, const char* link, OpenHRP::DynamicsSimulator::LinkDataType type, DblSequence_out rdata);
    virtual void getCharacterAllLinkData(const char* characterName, OpenHRP::DynamicsSimulator::LinkDataType type, DblSequence_out wdata);
    virtual void setCharacterAllLinkData(const char* characterName, OpenHRP::DynamicsSimulator::LinkDataType type, const DblSequence& wdata);
    virtual void setGVector(const DblSequence3& wdata);
    virtual void getGVector(DblSequence3_out wdata);
    virtual void setCharacterAllJointModes(const char* characterName, OpenHRP::DynamicsSimulator::JointDriveMode jointMode);
    virtual ::CORBA::Boolean calcCharacterInverseKinematics(const char* characterName, const char* baseLink, const char* targetLink, const LinkPosition& target);
    virtual void calcCharacterForwardKinematics(const char* characterName);
    virtual void calcWorldForwardKinematics();
    virtual void getWorldState(WorldState_out wstate);
    virtual void getCharacterSensorState(const char* characterName, SensorState_out sstate);
    virtual ::CORBA::Boolean getCharacterCollidingPairs(const char* characterName, LinkPairSequence_out pairs);
    virtual void calcCharacterJacobian(const char* characterName, const char* baseLink, const char* targetLink, DblSequence_out jacobian);

private:
    cnoid::BodyPtr body;

    cnoid::DeviceList<cnoid::ForceSensor> forceSensorIdMap;
    cnoid::DeviceList<cnoid::RateGyroSensor> gyroIdMap;
    cnoid::DeviceList<cnoid::AccelerationSensor> accelSensorIdMap;
};
}

#endif
    
