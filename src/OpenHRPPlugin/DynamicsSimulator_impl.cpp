/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "DynamicsSimulator_impl.h"
#include <cnoid/Link>

using namespace std;
using namespace cnoid;
using namespace OpenHRP;


DynamicsSimulator_impl::DynamicsSimulator_impl(const BodyPtr& body)
{
    this->body = body;

    for(auto joint : body->joints()){
        joint->setActuationMode(Link::JOINT_EFFORT);
    }
    
    forceSensorIdMap = body->devices<ForceSensor>().getSortedById();
    gyroIdMap = body->devices<RateGyroSensor>().getSortedById();
    accelSensorIdMap = body->devices<AccelerationSensor>().getSortedById();
}


DynamicsSimulator_impl::~DynamicsSimulator_impl()
{
    PortableServer::POA_var poa = _default_POA();
    PortableServer::ObjectId_var id = poa->servant_to_id(this);
    poa->deactivate_object(id);
}


void DynamicsSimulator_impl::destroy()
{

}


#ifdef OPENHRP_3_0
void DynamicsSimulator_impl::registerCharacter(const char* name, CharacterInfo_ptr cinfo)
{

}
#elif OPENHRP_3_1
void DynamicsSimulator_impl::registerCharacter(const char* name, BodyInfo_ptr cinfo)
{


}
#endif

#ifdef OPENHRP_3_0
void DynamicsSimulator_impl::registerCollisionCheckPair(
    const char* char1, const char* name1, const char* char2, const char* name2,
    ::CORBA::Double staticFriction, ::CORBA::Double slipFriction, const DblSequence6& K, const DblSequence6& C)
{

}
#elif OPENHRP_3_1
void DynamicsSimulator_impl::registerCollisionCheckPair(
    const char* char1, const char* name1, const char* char2, const char* name2,
    ::CORBA::Double staticFriction, ::CORBA::Double slipFriction, const DblSequence6& K, const DblSequence6& C, ::CORBA::Double culling_thresh)
{

}

void DynamicsSimulator_impl::registerIntersectionCheckPair(const char* char1, const char* name1, const char* char2, const char* name2, ::CORBA::Double tolerance)
{

}
#endif


void DynamicsSimulator_impl::init(
    ::CORBA::Double timeStep, OpenHRP::DynamicsSimulator::IntegrateMethod integrateOpt, OpenHRP::DynamicsSimulator::SensorOption sensorOpt)
{

}



void DynamicsSimulator_impl::registerVirtualLink(
    const char* char1, const char* link1, const char* char2, const char* link2,
    const LinkPosition& relTransform, ::CORBA::Short transformDefined, const DblSequence9& constraint, const char* connectionName)
{

}

    
void DynamicsSimulator_impl::getConnectionConstraintForce(
    const char* characterName, const char* connectionName, DblSequence6_out contactForce)
{

}


void DynamicsSimulator_impl::getCharacterSensorValues
(const char* character, const char* sensorName, DblSequence_out out_values)
{
    DblSequence_var values = new DblSequence;
    if(Device* sensor = body->findDevice(sensorName)){
        values->length(sensor->stateSize());
        sensor->writeState(&values[0]);
    }
    out_values = values._retn();
}

    
void DynamicsSimulator_impl::initSimulation()
{

}


void DynamicsSimulator_impl::stepSimulation()
{

}


void DynamicsSimulator_impl::setCharacterLinkData(
    const char* character, const char* linkName, OpenHRP::DynamicsSimulator::LinkDataType type, const DblSequence& data)
{
    Link* link = body->link(linkName);

    if(link){
        switch(type) {
            
        case OpenHRP::DynamicsSimulator::POSITION_GIVEN:
            if(link->isRoot()){
                link->setActuationMode(Link::LINK_POSITION);
            } else {
                link->setActuationMode(Link::JOINT_DISPLACEMENT);
            }
            break;
            
        case OpenHRP::DynamicsSimulator::JOINT_VALUE:
            if(!link->isFixedJoint()){
                link->q_target() = data[0];
            }
            break;
            
        case OpenHRP::DynamicsSimulator::JOINT_VELOCITY:
            if(!link->isFixedJoint() ||
               link->jointType() == Link::PSEUDO_CONTINUOUS_TRACK ||
               link->actuationMode() == Link::JOINT_SURFACE_VELOCITY){
                link->dq_target() = data[0];
            }
            break;
            
        case OpenHRP::DynamicsSimulator::JOINT_ACCELERATION:
            if(!link->isFixedJoint()){
                link->ddq() = data[0];
            }
            break;
            
        case OpenHRP::DynamicsSimulator::JOINT_TORQUE:
            if(!link->isFixedJoint()){
                link->u() = data[0];
            }
            break;
            
        case OpenHRP::DynamicsSimulator::ABS_TRANSFORM:
        {
            link->p() = Eigen::Map<const Vector3>(&data[0]);
            Matrix3 R;
            R << data[3], data[4],  data[5],
                data[6], data[7],  data[8],
                data[9], data[10], data[11];
            link->setAttitude(R);
            break;
        }

        case OpenHRP::DynamicsSimulator::ABS_VELOCITY:
        {
            link->v() = Eigen::Map<const Vector3>(&data[0]);
            link->w() = Eigen::Map<const Vector3>(&data[3]);
            break;
        }
        
        case OpenHRP::DynamicsSimulator::EXTERNAL_FORCE:
        {
            link->f_ext()   = Eigen::Map<const Vector3>(&data[0]);
            link->tau_ext() = Eigen::Map<const Vector3>(&data[3]);
            break;
        }

        default:
            break;
        }
    }
}


void DynamicsSimulator_impl::getCharacterLinkData(
    const char* characterName, const char* linkName, OpenHRP::DynamicsSimulator::LinkDataType type, DblSequence_out out_rdata)
{
    const Link* link = body->link(linkName);

    DblSequence_var rdata = new DblSequence;

    if(link){
        switch(type) {
        case OpenHRP::DynamicsSimulator::JOINT_VALUE:
            rdata->length(1);
            rdata[0] = link->q();
            break;
            
        case OpenHRP::DynamicsSimulator::JOINT_VELOCITY:
            rdata->length(1);
            rdata[0] = link->dq();
            break;
            
        case OpenHRP::DynamicsSimulator::JOINT_ACCELERATION:
            rdata->length(1);
            rdata[0] = link->ddq();
            break;
            
        case OpenHRP::DynamicsSimulator::JOINT_TORQUE:
            rdata->length(1);
            rdata[0] = link->u();
            break;
            
        case OpenHRP::DynamicsSimulator::ABS_TRANSFORM:
        {
            rdata->length(12);
            rdata[0] = link->p().x();
            rdata[1] = link->p().y();
            rdata[2] = link->p().z();
            const Matrix3 R = link->attitude();
            for(int i=0; i < 3; ++i){
                for(int j=0; j < 3; ++j){
                    rdata[3 + i * 3 + j] = R(i, j);
                }
            }
        }
        break;
        
        case OpenHRP::DynamicsSimulator::ABS_VELOCITY:
            rdata->length(6);
            rdata[0] = link->v().x();
            rdata[1] = link->v().y();
            rdata[2] = link->v().z();
            rdata[3] = link->w().x();
            rdata[4] = link->w().y();
            rdata[5] = link->w().z();
            break;
            
        case OpenHRP::DynamicsSimulator::EXTERNAL_FORCE:
            rdata->length(6);
            rdata[0] = link->f_ext().x();
            rdata[1] = link->f_ext().y();
            rdata[2] = link->f_ext().z();
            rdata[3] = link->tau_ext().x();
            rdata[4] = link->tau_ext().y();
            rdata[5] = link->tau_ext().z();
            break;
            
        default:
            break;
        }
    }
        
    out_rdata = rdata._retn();
}


void DynamicsSimulator_impl::getCharacterAllLinkData(
    const char* characterName, OpenHRP::DynamicsSimulator::LinkDataType type, DblSequence_out rdata)
{
    const int n = body->numJoints();
    rdata = new DblSequence();
    rdata->length(n);

    switch(type) {

    case OpenHRP::DynamicsSimulator::JOINT_VALUE:
        for(int i=0; i < n; ++i){
            (*rdata)[i] = body->joint(i)->q();
        }
        break;

    case OpenHRP::DynamicsSimulator::JOINT_VELOCITY:
        for(int i=0; i < n; ++i){
            (*rdata)[i] = body->joint(i)->dq();
        }
        break;
        
    case OpenHRP::DynamicsSimulator::JOINT_ACCELERATION:
        for(int i=0; i < n; ++i){
            (*rdata)[i] = body->joint(i)->ddq();
        }
        break;
        
    case OpenHRP::DynamicsSimulator::JOINT_TORQUE:
        for(int i=0; i < n; ++i){
            (*rdata)[i] = body->joint(i)->u();
        }
        break;
        
    default:
        // put error here
        break;
    }
}


void DynamicsSimulator_impl::setCharacterAllLinkData(
    const char* characterName, OpenHRP::DynamicsSimulator::LinkDataType type, const DblSequence& wdata)
{
    int n = wdata.length();
    if(n > body->numJoints()){
        n = body->numJoints();
    }

    switch(type) {
        
    case OpenHRP::DynamicsSimulator::JOINT_VALUE:
        for(int i=0; i < n; ++i){
            body->joint(i)->q_target() = wdata[i];
        }
        break;
        
    case OpenHRP::DynamicsSimulator::JOINT_VELOCITY:
        for(int i=0; i < n; ++i){
            body->joint(i)->dq_target() = wdata[i];
        }
        break;
        
    case OpenHRP::DynamicsSimulator::JOINT_ACCELERATION:
        for(int i=0; i < n; ++i){
            body->joint(i)->ddq() = wdata[i];
        }
        break;
        
    case OpenHRP::DynamicsSimulator::JOINT_TORQUE:
        for(int i=0; i < n; ++i){
            body->joint(i)->u() = wdata[i];
        }
        break;
        
    default:
        // put error here
        break;
    }
}


void DynamicsSimulator_impl::setGVector(const DblSequence3& wdata)
{

}


void DynamicsSimulator_impl::getGVector(DblSequence3_out wdata)
{

}


void DynamicsSimulator_impl::setCharacterAllJointModes(const char* characterName, OpenHRP::DynamicsSimulator::JointDriveMode jointMode)
{
    Link::ActuationMode actuationMode = Link::JOINT_EFFORT;
    if(jointMode == OpenHRP::DynamicsSimulator::HIGH_GAIN_MODE){
        actuationMode = Link::JOINT_DISPLACEMENT;
    }
    for(auto joint : body->joints()){
        joint->setActuationMode(actuationMode);
    }
}


::CORBA::Boolean DynamicsSimulator_impl::calcCharacterInverseKinematics(
    const char* characterName, const char* baseLink, const char* targetLink, const LinkPosition& target)
{
    return true;
}


void DynamicsSimulator_impl::calcCharacterForwardKinematics(const char* characterName)
{

}


void DynamicsSimulator_impl::calcWorldForwardKinematics()
{

}


#ifdef OPENHRP_3_0
void DynamicsSimulator_impl::checkCollision()
{

}

#elif OPENHRP_3_1
::CORBA::Boolean DynamicsSimulator_impl::checkCollision(::CORBA::Boolean checkAll)
{
    return false;
}
LinkPairSequence* DynamicsSimulator_impl::checkIntersection(::CORBA::Boolean checkAll)
{
    return 0;
}
DistanceSequence* DynamicsSimulator_impl::checkDistance()
{
    return 0;
}
#endif


void DynamicsSimulator_impl::getWorldState(WorldState_out wstate)
{

}


void DynamicsSimulator_impl::getCharacterSensorState(const char* characterName, SensorState_out sstate)
{
    sstate = new SensorState;
    
    int numJoints = body->numJoints();
    sstate->q.length(numJoints);
    sstate->dq.length(numJoints);
    sstate->u.length(numJoints);
    sstate->force.length(forceSensorIdMap.size());
    sstate->rateGyro.length(gyroIdMap.size());
    sstate->accel.length(accelSensorIdMap.size());
    
    for(int j=0; j < numJoints; j++){
        Link* joint = body->joint(j);
        sstate->q [j] = joint->q();
        sstate->dq[j] = joint->dq();
        sstate->u [j] = joint->u();
    }

    for(size_t id = 0; id < forceSensorIdMap.size(); ++id){
        Eigen::Map<Vector6>(&sstate->force[id][0]) = forceSensorIdMap[id]->F();
    }
    for(size_t id = 0; id < gyroIdMap.size(); ++id){
        Eigen::Map<Vector3>(sstate->rateGyro[id]) = gyroIdMap[id]->w();
    }
    for(size_t id = 0; id < accelSensorIdMap.size(); ++id){
        Eigen::Map<Vector3>(sstate->accel[id]) = accelSensorIdMap[id]->dv();
    }
}


::CORBA::Boolean DynamicsSimulator_impl::getCharacterCollidingPairs(const char* characterName, LinkPairSequence_out pairs)
{
    return true;
}


void DynamicsSimulator_impl::calcCharacterJacobian(
    const char* characterName, const char* baseLink, const char* targetLink, DblSequence_out jacobian)
{

}
