/**
   \file
   \author Ikumi Susa
*/

#include <cnoid/Device>
#include <cnoid/YAMLBodyLoader>
#include <cnoid/YAMLReader>
#include <cnoid/AGXBodyExtension>
#include <cnoid/AGXBody>
#include <cnoid/AGXScene>

using namespace std;
namespace cnoid{

/////////////////////////////////////////////////////////////////////////
// AGXBreakableJointDevice
struct AGXBreakableJointDeviceDesc
{
    AGXBreakableJointDeviceDesc(){
        breakLimitForce = std::numeric_limits<double>::max();
        jointType = "revolute";
    }
    std::string     link1Name;
    std::string     link2Name;
    Vector3         link1LocalPos;
    Vector3         link2LocalPos;
    std::string     jointType;
    Vector3         jointAxis;
    double          breakLimitForce;
};

class AGXBreakableJointDevice : private AGXBreakableJointDeviceDesc, public Device
{
public:
    static bool createAGXBreakableJointDevice(YAMLBodyLoader& loader, Mapping& node);
    AGXBreakableJointDevice(const AGXBreakableJointDeviceDesc& desc);
    AGXBreakableJointDevice(const AGXBreakableJointDevice& org, bool copyStateOnly = false);
    virtual const char* typeName() override;
    void copyStateFrom(const AGXBreakableJointDevice& other);
    virtual void copyStateFrom(const DeviceState& other) override;
    virtual DeviceState* cloneState() const override;
    virtual Device* clone() const override;
    virtual void forEachActualType(std::function<bool(const std::type_info& type)> func) override;
    virtual int stateSize() const override;
    virtual const double* readState(const double* buf) override;
    virtual double* writeState(double* out_buf) const override;

    void setDesc(const AGXBreakableJointDeviceDesc& desc);
    void getDesc(AGXBreakableJointDeviceDesc& desc);
    //void initialize();
};
typedef ref_ptr<AGXBreakableJointDevice> AGXBreakableJointDevicePtr;

#define NODE_READ(FIELD1)    node.read(#FIELD1, desc.FIELD1)
#define NODE_TO_VEC3(FIELD1) valueNodeToVec3(info->extract(#FIELD1), desc.FIELD1);
bool AGXBreakableJointDevice::createAGXBreakableJointDevice(YAMLBodyLoader&loader, Mapping&node)
{
    auto valueNodeToVec3 = [](ValueNodePtr vnode, Vector3&vec)
    {
        if(!vnode){
            cout << "NOt Correct Vec3!" << std::endl;
            return false;
        }
        Listing&u = *vnode->toListing();
        if(u.size() != 3){
            cout << "Not correct Vec3!" << std::endl;
            return false;
        }
        vec = Vector3(u[0].toDouble(), u[1].toDouble(), u[2].toDouble());
        return true;
    };

    MappingPtr info = static_cast<Mapping*>(node.clone());
    AGXBreakableJointDeviceDesc desc;
    NODE_READ(breakLimitForce);
    NODE_READ(jointType);
    NODE_READ(link1Name);
    NODE_READ(link2Name);
    NODE_TO_VEC3(link1LocalPos);
    NODE_TO_VEC3(link2LocalPos);
    NODE_TO_VEC3(jointAxis);

    AGXBreakableJointDevicePtr jointDevice = new AGXBreakableJointDevice(desc);
    return loader.readDevice(jointDevice, node);
}
#undef NODE_READ
#undef NODE_TO_VEC3

AGXBreakableJointDevice::AGXBreakableJointDevice(const AGXBreakableJointDeviceDesc& desc) :
    AGXBreakableJointDeviceDesc(desc)
{
}

AGXBreakableJointDevice::AGXBreakableJointDevice(const AGXBreakableJointDevice& org, bool copyStateOnly) :
    Device(org, copyStateOnly)
{
    copyStateFrom(org);
}

const char*AGXBreakableJointDevice::typeName()
{
    return "AGXBreakableJointDevice";
}

void AGXBreakableJointDevice::copyStateFrom(const AGXBreakableJointDevice& other)
{
    AGXBreakableJointDeviceDesc desc;
    AGXBreakableJointDevice& device = const_cast<AGXBreakableJointDevice&>(other);
    device.getDesc(desc);  // Need to get desc. So do const_cast above.
    setDesc(desc);
}

void AGXBreakableJointDevice::copyStateFrom(const DeviceState&other)
{
    if(typeid(other) != typeid(AGXBreakableJointDevice)){
        throw std::invalid_argument("Type mismatch in the Device::copyStateFrom function");
    }
    copyStateFrom(static_cast<const AGXBreakableJointDevice&>(other));
}

DeviceState* AGXBreakableJointDevice::cloneState() const
{
    return new AGXBreakableJointDevice(*this, false);
}

Device*AGXBreakableJointDevice::clone() const
{
    return new AGXBreakableJointDevice(*this);
}

void AGXBreakableJointDevice::forEachActualType(std::function<bool(const std::type_info&type)> func)
{
    if(!func(typeid(AGXBreakableJointDevice))){
        Device::forEachActualType(func);
    }
}

int AGXBreakableJointDevice::stateSize() const
{
    return 1;
}

const double* AGXBreakableJointDevice::readState(const double* buf)
{
    return buf + 1;
}

double* AGXBreakableJointDevice::writeState(double* out_buf) const
{
    return out_buf + 1;
}

void AGXBreakableJointDevice::setDesc(const AGXBreakableJointDeviceDesc& desc)
{
    static_cast<AGXBreakableJointDeviceDesc&>(*this) = desc;
}

void AGXBreakableJointDevice::getDesc(AGXBreakableJointDeviceDesc& desc)
{
    desc = static_cast<AGXBreakableJointDeviceDesc&>(*this);
}

/////////////////////////////////////////////////////////////////////////
// JointBreaker
class JointBreaker : public agxSDK::StepEventListener
{
private:
    agx::Constraint1DOFObserver m_joint;
    double m_breakLimitForce;

public:
    JointBreaker(agx::Constraint1DOF* joint, double breakLimitForce) :
        m_joint(joint),
        m_breakLimitForce(breakLimitForce)
    {
        m_joint->setEnableComputeForces(true);
    }

   virtual void post( const agx::TimeStamp& /*t*/ )
    {
        agx::Vec3 force;
        agx::Vec3 torque;
        m_joint->getLastForce((agx::UInt)0, force, torque);
        //force.set(0.0, 2);

        if(force.length() >= m_breakLimitForce){
            m_joint->setEnable(false);
        }
    }
};

/////////////////////////////////////////////////////////////////////////
// AGXBreakableJoint
class AGXBreakableJoint : public AGXBodyExtension
{
public:
    static bool createAGXBreakableJoint(cnoid::AGXBody* agxBody);
    AGXBreakableJoint(AGXBreakableJointDevice* device, AGXBody* agxBody);
private:
    AGXBreakableJointDevicePtr m_device;
};
typedef ref_ptr<AGXBreakableJoint> AGXBreakableJointPtr;


bool AGXBreakableJoint::createAGXBreakableJoint(cnoid::AGXBody* agxBody)
{
    DeviceList<> devices = agxBody->body()->devices();
    DeviceList<AGXBreakableJointDevice> jointDevices;
    jointDevices.extractFrom(devices);
    for(auto device : jointDevices){
        agxBody->addAGXBodyExtension(new cnoid::AGXBreakableJoint(device, agxBody));
    }
    return true;
}

AGXBreakableJoint::AGXBreakableJoint(AGXBreakableJointDevice* device, AGXBody* agxBody) :
    AGXBodyExtension(agxBody)
{
    AGXBreakableJointDeviceDesc ddesc;
    device->getDesc(ddesc);
    Link* const link1 = getAGXBody()->getAGXLink(ddesc.link1Name)->getOrgLink();
    const Vector3 p = link1->attitude() * ddesc.link1LocalPos + link1->p();
    const Vector3 a = link1->attitude() * ddesc.jointAxis;

    AGXHingeDesc hd;
    hd.rigidBodyA = getAGXBody()->getAGXRigidBody(ddesc.link1Name);
    hd.rigidBodyB = getAGXBody()->getAGXRigidBody(ddesc.link2Name);
    hd.frameAxis = agx::Vec3(a(0), a(1), a(2));
    hd.frameCenter = agx::Vec3(p(0), p(1), p(2));
    agx::ConstraintRef joint = AGXObjectFactory::createConstraint(hd);
    double c = 1e-3;
    joint->setCompliance(c, agx::Hinge::TRANSLATIONAL_1);
    joint->setCompliance(c, agx::Hinge::TRANSLATIONAL_2);
    joint->setCompliance(c, agx::Hinge::ROTATIONAL_1);
    joint->setCompliance(c, agx::Hinge::ROTATIONAL_2);
    getAGXBody()->getAGXScene()->getSimulation()->add(joint);

    agx::Constraint1DOF* const joint1DOF = agx::Constraint1DOF::safeCast(joint);
    getAGXBody()->getAGXScene()->getSimulation()
        ->add(new JointBreaker(joint1DOF, ddesc.breakLimitForce));
}

} // cnoid

namespace{
using namespace cnoid;

/////////////////////////////////////////////////////////////////////////
// Register AGXBreakableJointDevice
struct AGXBreakableJointDeviceRegistration
{
    AGXBreakableJointDeviceRegistration()
    {
        YAMLBodyLoader::addNodeType("AGXBreakableJointDevice", AGXBreakableJointDevice::createAGXBreakableJointDevice);
    }
}registrationAGXBreakableJointDevice;

/////////////////////////////////////////////////////////////////////////
// Register AGXBreakableJoint
struct AGXBreakableJointRegistration
{
    AGXBreakableJointRegistration() {
        AGXBody::addAGXBodyExtensionAdditionalFunc("AGXBreakableJoint", AGXBreakableJoint::createAGXBreakableJoint);
    }
}registrationAGXBreakableJoint;

}